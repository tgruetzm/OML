
#include "Arduino.h"
#include <GGOW4_inferencing.h>
#include <PDM.h> //Include PDM library included with the Aruino_Apollo3 core
//#include "SD.h"
#include "SdFat.h"
#include <SPI.h>
#include "RTC.h" // Include RTC library included with the Aruino_Apollo3 core
#include "BurstMode.h"

#define SPI_SPEED SD_SCK_MHZ(50)
SdFat SD;
AP3_PDM myPDM;   //Create instance of PDM class with our buffer

#define pdmDataSize 8000 //must change based on sample frequency //Library requires array be 4096
uint16_t pdmData[pdmDataSize];
/** Audio buffers, pointers and selectors */
typedef struct {
    int16_t *buffer;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

int MODE_BUTTON_PIN = 10;
enum Mode { adhoc, passive};
//default mode
Mode mode = passive;

int sd_failure = false;

void *PDMHandle = NULL;
am_hal_pdm_config_t newConfig = {
    .eClkDivider = AM_HAL_PDM_MCLKDIV_1,
    .eLeftGain = AM_HAL_PDM_GAIN_P405DB,
    .eRightGain = AM_HAL_PDM_GAIN_P405DB, //Found empirically
    .ui32DecimationRate = 48,            // OSR = 1500/16 = 96 = 2*SINCRATE --> SINC_RATE = 48
    .bHighPassEnable = 0,//this is actuall enabled
    .ui32HighPassCutoff = 0x9,//0x9 seems to filter out low band noise well, need to test if we are attenuating the signal too far
    .ePDMClkSpeed = AM_HAL_PDM_CLK_750KHZ,//16khzAM_HAL_PDM_CLK_1_5MHZ,
    .bInvertI2SBCLK = 0,
    .ePDMClkSource = AM_HAL_PDM_INTERNAL_CLK,
    .bPDMSampleDelay = 0,
    .bDataPacking = 1,
    .ePCMChannels = AM_HAL_PDM_CHANNEL_RIGHT,
    .ui32GainChangeDelay = 1,
    .bI2SEnable = 0,
    .bSoftMute = 0,
    .bLRSwap = 0,
};

/*void dateTime(uint16_t* date, uint16_t* time) {
    rtc.getTime();
    *date = FAT_DATE(rtc.year + 2000, rtc.month, rtc.dayOfMonth);
    *time = FAT_TIME(rtc.hour, rtc.minute, rtc.seconds);
}*/

//needs some optimization but works
String getFileName(String base, uint32_t &epoch)
{
    if(!SD.exists("/audio"))
        SD.mkdir("/audio");
    rtc.getTime();
    epoch = rtc.getEpoch();
    String name = "/audio/"+ base + String(epoch) + ".raw";
  return name;
}

static inference_t inference;
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

void recordInference()
{
    Serial.print("Initializing SD card...");
    if (!SD.begin(2)) {
        Serial.println("initialization failed!");
        sd_failure = true;
        return;
    }
    sd_failure = false;
    Serial.println("initialization done.");
    uint32_t epoch;
    String filename = getFileName("inf", epoch);
    File myFile = SD.open(filename,  FILE_WRITE);
    myFile.timestamp(T_CREATE | T_WRITE, rtc.year + 2000, rtc.month, rtc.dayOfMonth, rtc.hour, rtc.minute, rtc.seconds);
    myFile.write((uint8_t *)pdmData, sizeof(pdmData));
    myFile.close();
    
}

void blinkIndicator()
{
    for(int i = 0; i < 5; i++)
    {
        digitalWrite(LED_BUILTIN,LOW);
        delay(200);
        digitalWrite(LED_BUILTIN,HIGH);
        delay(200);
    }
}

void writeLog(String message)
{
    File file = SD.open("log.txt", FILE_WRITE);
    file.println(message);
    file.close();
}
/**
 * @brief      Init inferencing struct and setup/start PDM
 *
 * @param[in]  n_samples  The n samples
 *
 * @return     { description_of_the_return_value }
 */
bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));
    inference.n_samples  = n_samples;

    if (myPDM.begin() == false) // Turn on PDM with default settings, start interrupts
    {
        Serial.println("PDM Init failed. Are you sure these pins are PDM capable?");
        return false;
    }
    bool output = myPDM.updateConfig(newConfig); //Send config struct
    return true;
}


/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);

    return 0;
}

/**
 * @brief      Arduino setup function
 */
void setup()
{
    Serial.begin(115200);
    rtc.getTime();
    //rtc.setTime(0, 0, 34, 8, 9, 7, 22); // (hund, ss, mm, hh, dd, mm, yy)
    char buffer[40];
    sprintf(buffer,"Now: %02d:%02d:%02d %02d/%02d/%02d",rtc.hour,rtc.minute,rtc.seconds, rtc.month,rtc.dayOfMonth,rtc.year);
    Serial.println(buffer);
    //SdFile::dateTimeCallback(dateTime);
    //validate SD is working
    Serial.print("Initializing SD card...");
    if (!SD.begin(2))
    { 
        Serial.println("initialization failed!");
        sd_failure = true;
    }
    else
        SD.end();
    
    //enableBurstMode(); //Go to 96MHz
    //LED indicator for file writing/GGO detection
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    //mode button
    pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);


    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        ei_printf("ERR: Failed to setup audio sampling\r\n");
        return;
    }
}

/**
 * @brief      Arduino main function. Runs the inferencing loop.
 */
void loop()
{
    if(sd_failure)
        blinkIndicator();
    //check mode
    if(digitalRead(MODE_BUTTON_PIN) == LOW)
    {
        mode = adhoc;
        blinkIndicator();
    }

    if(mode == adhoc){
         Serial.print("Initializing SD card...");
        if (!SD.begin(2))
        { 
            Serial.println("initialization failed!");
            sd_failure = true;
        }
        else
        {
            sd_failure = false;
            uint32_t epoch;
            String filename = getFileName("adhoc",epoch);
            File myFile = SD.open(filename,  FILE_WRITE);
            myFile.timestamp(T_CREATE | T_WRITE, rtc.year + 2000, rtc.month, rtc.dayOfMonth, rtc.hour, rtc.minute, rtc.seconds);
            Serial.print("writing to file:");
            Serial.println(filename);
            while(digitalRead(MODE_BUTTON_PIN) != LOW)
            {
                if(myPDM.available())
                {
                    myPDM.getData(pdmData, pdmDataSize);
                    myFile.write((uint8_t *)pdmData, sizeof(pdmData));
                }
            }
            blinkIndicator();
            digitalWrite(LED_BUILTIN,LOW);
            myFile.close();
            Serial.println("done recording to file");
        }
        mode = passive;
    }
    else if(mode == passive)
    {
        inference.buf_count = 0;
        while(inference.buf_count < inference.n_samples)
        {
            if(myPDM.available())
            {
                int available = myPDM.getData(pdmData, pdmDataSize);
                for(int i = 0; i < available; i++) {
                    inference.buffer[inference.buf_count++] = pdmData[i];
                }
            }
        }

        signal_t signal;
        signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
        signal.get_data = &microphone_audio_signal_get_data;
        ei_impulse_result_t result = { 0 };

        EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
        if (r != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", r);
            return;
        }
        digitalWrite(LED_BUILTIN, LOW);
        // print the predictions
        ei_printf("Predictions ");
        ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
        ei_printf(": \n");
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            //ei_printf("    %s: %f\n", result.classification[ix].label, result.classification[ix].value);
            if(strcmp("GGOW Territorial", result.classification[ix].label) == 0 && result.classification[ix].value > 0.5)
            {
                digitalWrite(LED_BUILTIN, HIGH);
                Serial.println("Recording inference sample to file...");
                recordInference();
                char buffer[40];
                sprintf(buffer,"Now: %02d:%02d:%02d %02d/%02d/%02d",rtc.hour,rtc.minute,rtc.seconds, rtc.month,rtc.dayOfMonth,rtc.year);
                writeLog(buffer);
                sprintf(buffer,"Epoch: %d",rtc.getEpoch());
                writeLog(buffer);
                char x[8];
                float val = result.classification[ix].value;
                sprintf(buffer, "    %s: %d.%02d", result.classification[ix].label,  (int)val, (int)(fabsf(val)*100)%100);
                writeLog(buffer);
            }
            Serial.print(result.classification[ix].label);
            Serial.println((result.classification[ix].value));
        }
    }
}

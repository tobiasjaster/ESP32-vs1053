/*
 *  vs1053.h
 *
 *  Created on: Jul 09.2017
 *  Updated on: Aug 21 2021
 *      Author: Wolle
 */

#pragma once

#include "Arduino.h"
#include "libb64/cencode.h"
#include "SPI.h"
#include "SD.h"
#include "SD_MMC.h"
#include "SPIFFS.h"
#include "FS.h"
#include "FFat.h"
#include "WiFiClient.h"
#include "WiFiClientSecure.h"

#include "vs1053b-patches-flac.h"

extern __attribute__((weak)) void vs1053_info(const char*);
extern __attribute__((weak)) void vs1053_showstreamtitle(const char*);
extern __attribute__((weak)) void vs1053_showstation(const char*);
extern __attribute__((weak)) void vs1053_showstreaminfo(const char*);
extern __attribute__((weak)) void vs1053_id3data(const char*); //ID3 metadata
extern __attribute__((weak)) void vs1053_id3image(File& file, const size_t pos, const size_t size); //ID3 metadata image
extern __attribute__((weak)) void vs1053_eof_mp3(const char*);
extern __attribute__((weak)) void vs1053_eof_speech(const char*);
extern __attribute__((weak)) void vs1053_bitrate(const char*);
extern __attribute__((weak)) void vs1053_commercial(const char*);
extern __attribute__((weak)) void vs1053_icyurl(const char*);
extern __attribute__((weak)) void vs1053_icydescription(const char*);
extern __attribute__((weak)) void vs1053_lasthost(const char*);
extern __attribute__((weak)) void vs1053_eof_stream(const char*); // The webstream comes to an end

//----------------------------------------------------------------------------------------------------------------------

class AudioBuffer {
// AudioBuffer will be allocated in PSRAM, If PSRAM not available or has not enough space AudioBuffer will be
// allocated in FlashRAM with reduced size
//
//  m_buffer            m_readPtr                 m_writePtr                 m_endPtr
//   |                       |<------dataLength------->|<------ writeSpace ----->|
//   ▼                       ▼                         ▼                         ▼
//   ---------------------------------------------------------------------------------------------------------------
//   |                     <--m_buffSize-->                                      |      <--m_resBuffSize -->     |
//   ---------------------------------------------------------------------------------------------------------------
//   |<-----freeSpace------->|                         |<------freeSpace-------->|
//
//
//
//   if the space between m_readPtr and buffend < m_resBuffSize copy data from the beginning to resBuff
//   so that the mp3/aac/flac frame is always completed
//
//  m_buffer                      m_writePtr                 m_readPtr        m_endPtr
//   |                                 |<-------writeSpace------>|<--dataLength-->|
//   ▼                                 ▼                         ▼                ▼
//   ---------------------------------------------------------------------------------------------------------------
//   |                        <--m_buffSize-->                                    |      <--m_resBuffSize -->     |
//   ---------------------------------------------------------------------------------------------------------------
//   |<---  ------dataLength--  ------>|<-------freeSpace------->|
//
//

public:
    AudioBuffer(size_t maxBlockSize = 0);       // constructor
    ~AudioBuffer();                             // frees the buffer
    size_t   init();                            // set default values
    void     changeMaxBlockSize(uint16_t mbs);  // is default 1600 for mp3 and aac, set 16384 for FLAC
    uint16_t getMaxBlockSize();                 // returns maxBlockSize
    size_t   freeSpace();                       // number of free bytes to overwrite
    size_t   writeSpace();                      // space fom writepointer to bufferend
    size_t   bufferFilled();                    // returns the number of filled bytes
    void     bytesWritten(size_t bw);           // update writepointer
    void     bytesWasRead(size_t br);           // update readpointer
    uint8_t* getWritePtr();                     // returns the current writepointer
    uint8_t* getReadPtr();                      // returns the current readpointer
    uint32_t getWritePos();                     // write position relative to the beginning
    uint32_t getReadPos();                      // read position relative to the beginning
    void     resetBuffer();                     // restore defaults

protected:
    const size_t m_buffSizePSRAM    = 300000;   // most webstreams limit the advance to 100...300Kbytes
    const size_t m_buffSizeRAM      = 1600 * 10;
    size_t       m_buffSize         = 0;
    size_t       m_freeSpace        = 0;
    size_t       m_writeSpace       = 0;
    size_t       m_dataLength       = 0;
    size_t       m_resBuffSizeRAM   = 4096;     // reserved buffspace, >= one mp3  frame
    size_t       m_resBuffSizePSRAM = 4096;
    size_t       m_maxBlockSize     = 1600;
    uint8_t*     m_buffer           = NULL;
    uint8_t*     m_writePtr         = NULL;
    uint8_t*     m_readPtr          = NULL;
    uint8_t*     m_endPtr           = NULL;
    bool         m_f_start          = true;
};
//----------------------------------------------------------------------------------------------------------------------

class VS1053 : private AudioBuffer{

    AudioBuffer InBuff; // instance of input buffer

public:
    // Constructor.  Only sets pin values.  Doesn't touch the chip.  Be sure to call begin()!
    VS1053 ( uint8_t _cs_pin, uint8_t _dcs_pin, uint8_t _dreq_pin ) ;
    VS1053 ( uint8_t _cs_pin, uint8_t _dcs_pin, uint8_t _dreq_pin, SPIClass *_spi_bus ) ;
    VS1053 ( uint8_t _cs_pin, uint8_t _dcs_pin, uint8_t _dreq_pin, SPIClass *_spi_bus, fs::FS *_file_system) ;
    ~VS1053();

    void     begin() ;                                  // Begin operation.  Sets pins correctly,
                                                        // and prepares SPI bus.
    bool     connecttohost(String host);
    bool     connecttohost(const char* host, const char* user = "", const char* pwd = "");
    bool	 connecttoSD(String sdfile);
    bool     connecttoSD(const char* sdfile);
    bool     connecttoFS(fs::FS *fs, const char* path);
    bool     connecttospeech(const char* speech, const char* lang);
    bool     setFileLoop(bool input);
    bool     setAudioPlayPosition(uint16_t sec);
    bool     setFilePos(uint32_t pos);
    bool     audioFileSeek(const float speed);
    bool     setTimeOffset(int sec);
    bool     pauseResume();
    bool     isRunning() {return m_f_running;}
    void 	 loop();
    void     startSong() ;                              // Prepare to start playing. Call this each
                                                        // time a new song starts.
    void     stopSong() ;                               // Finish playing a song. Call this after
                                                        // the last playChunk call.
    void     stop_mp3client();
    void     setVolume(uint8_t vol);                    // Set the player volume.Level from 0-21, higher is louder.
    uint8_t  getVolume();                               // Get the current volume setting, higher is louder.

    uint32_t getAudioDataStartPos();
    uint32_t getFileSize();
    uint32_t getFilePos();
//  uint32_t getSampleRate();
//  uint8_t  getBitsPerSample();    
//  uint8_t  getChannels();
    uint32_t getBitRate();
//  uint32_t getAudioFileDuration();
    uint32_t getAudioCurrentTime();
    uint32_t getTotalPlayingTime();

    size_t   inBufferFilled();
    size_t   inBufferFree();
    void     setTone(uint8_t* rtone);                   // Set the player baas/treble, 4 nibbles for treble gain/freq and bass gain/freq
    void     printDetails(const char* str);             // Print configuration details to serial output.
    bool     printVersion();                            // Print ID and version of vs1053 chip
    void     softReset() ;                              // Do a soft reset

private:
    inline void DCS_HIGH() {(dcs_pin&0x20) ? GPIO.out1_w1ts.data = 1 << (dcs_pin - 32) : GPIO.out_w1ts = 1 << dcs_pin;}
	inline void DCS_LOW()  {(dcs_pin&0x20) ? GPIO.out1_w1tc.data = 1 << (dcs_pin - 32) : GPIO.out_w1tc = 1 << dcs_pin;}
	inline void CS_HIGH()  {( cs_pin&0x20) ? GPIO.out1_w1ts.data = 1 << ( cs_pin - 32) : GPIO.out_w1ts = 1 <<  cs_pin;}
    inline void CS_LOW()   {( cs_pin&0x20) ? GPIO.out1_w1tc.data = 1 << ( cs_pin - 32) : GPIO.out_w1tc = 1 <<  cs_pin;}
    inline void await_data_request() {while(!digitalRead(dreq_pin)) NOP();}	  // Very short delay
    inline bool data_request()     {return(digitalRead(dreq_pin) == HIGH);}

    void     UTF8toASCII(char* str);
    bool     latinToUTF8(char* buff, size_t bufflen);

    void     setDefaults();
    void     initInBuff();
    void     processLocalFile();
    void     processWebStream();
    void     processPlayListData();
//    void processM3U8entries(uint8_t nrOfEntries = 0, uint32_t seqNr = 0, uint8_t pos = 0, uint16_t targetDuration = 0);
    size_t   sendBytes(uint8_t* data, size_t len);
    void     showID3Tag(const char* tag, const char* value);
    void     unicode2utf8(char* buff, uint32_t len);
//    int      read_WAV_Header(uint8_t* data, size_t len);
//    int      read_FLAC_Header(uint8_t *data, size_t len);
    int      read_MP3_Header(uint8_t *data, size_t len);
//    int      read_M4A_Header(uint8_t* data, size_t len);
//    int      read_OGG_Header(uint8_t *data, size_t len);
    void     showstreamtitle(const char* ml);
    bool     parseContentType(const char* ct);
    void     processAudioHeaderData();
    void     urlencode(char* buff, uint16_t buffLen, bool spacesOnly = false);

    void     control_mode_on();
    void     control_mode_off();
    void     data_mode_on();
    void     data_mode_off();
    uint16_t read_register ( uint8_t _reg ) ;
    void     write_register ( uint8_t _reg, uint16_t _value );
    void     sdi_send_buffer ( uint8_t* data, size_t len ) ;
    void     sdi_send_fillers ( size_t length ) ;
    void     wram_write ( uint16_t address, uint16_t data ) ;
    uint16_t wram_read ( uint16_t address ) ;
    bool     readMetadata(uint8_t b, bool first = false);
    bool     doNotJumpSet();
    void     loadUserCode();
    void     compute_bitrate();

    // implement several function with respect to the index of string
    bool startsWith (const char* base, const char* str) { return (strstr(base, str) - base) == 0;}
    bool endsWith (const char* base, const char* str) {
        int blen = strlen(base);
        int slen = strlen(str);
        return (blen >= slen) && (0 == strcmp(base + blen - slen, str));
    }
    int indexOf (const char* base, const char* str, int startIndex) {
        int result;
        int baselen = strlen(base);
        if (strlen(str) > baselen || startIndex > baselen) result = -1;
        else {
            char* pos = strstr(base + startIndex, str);
            if (pos == NULL) result = -1;
            else result = pos - base;
        }
        return result;
    }
    int lastIndexOf(const char* base, const char* str) {
        int res = -1, result = -1;
        int lenBase = strlen(base);
        int lenStr  = strlen(str);
        if(lenStr > lenBase) {return -1;} // str should not longer than base
        for(int i=0; i<(lenBase - lenStr); i++){
            res = indexOf(base, str, i);
            if(res > result) result = res;
        }
        return result;
    }
    int specialIndexOf (uint8_t* base, const char* str, int baselen, bool exact = false){
        int result;  // seek for str in buffer or in header up to baselen, not nullterninated
        if (strlen(str) > baselen) return -1; // if exact == true seekstr in buffer must have "\0" at the end
        for (int i = 0; i < baselen - strlen(str); i++){
            result = i;
            for (int j = 0; j < strlen(str) + exact; j++){
                if (*(base + i + j) != *(str + j)){
                    result = -1;
                    break;
                }
            }
            if (result >= 0) break;
        }
        return result;
    }
    size_t bigEndian(uint8_t* base, uint8_t numBytes, uint8_t shiftLeft = 8){
        size_t result = 0;
        if(numBytes < 1 or numBytes > 4) return 0;
        for (int i = 0; i < numBytes; i++) {
                result += *(base + i) << (numBytes -i - 1) * shiftLeft;
        }
        return result;
    }
    bool b64encode(const char* source, uint16_t sourceLength, char* dest){
        size_t size = base64_encode_expected_len(sourceLength) + 1;
        char * buffer = (char *) malloc(size);
        if(buffer) {
            base64_encodestate _state;
            base64_init_encodestate(&_state);
            int len = base64_encode_block(&source[0], sourceLength, &buffer[0], &_state);
            len = base64_encode_blockend((buffer + len), &_state);
            memcpy(dest, buffer, strlen(buffer));
            dest[strlen(buffer)] = '\0';
            free(buffer);
            return true;
        }
        return false;
    }
    size_t urlencode_expected_len(const char* source){
        size_t expectedLen = strlen(source);
        for(int i = 0; i < strlen(source); i++) {
            if(isalnum(source[i])){;}
            else expectedLen += 2;
        }
        return expectedLen;
    }
    void trim(char* s){
        uint8_t l = 0;
        while(isspace(*(s + l))) l++;
        for(uint16_t i = 0; i< strlen(s) - l; i++)  *(s + i) = *(s + i + l); // ltrim
        char* back = s + strlen(s);
        while(isspace(*--back));
        *(back + 1) = '\0';      // rtrim
    }


    inline uint8_t  getDatamode(){return m_datamode;}
    inline void     setDatamode(uint8_t dm){m_datamode=dm;}
    inline uint32_t streamavail() {if(m_f_ssl==false) return client.available(); else return clientsecure.available();}


private:
    enum : int { VS1053_NONE, VS1053_HEADER , VS1053_DATA, VS1053_METADATA, VS1053_PLAYLISTINIT,
                 VS1053_PLAYLISTHEADER,  VS1053_PLAYLISTDATA, VS1053_SWM, VS1053_OGG};
    enum : int { FORMAT_NONE = 0, FORMAT_M3U = 1, FORMAT_PLS = 2, FORMAT_ASX = 3, FORMAT_M3U8 = 4};

    enum : int { CODEC_NONE, CODEC_WAV, CODEC_MP3, CODEC_AAC, CODEC_M4A, CODEC_FLAC, CODEC_OGG,
                 CODEC_OGG_FLAC, CODEC_OGG_OPUS};
    enum : int { FLAC_BEGIN = 0, FLAC_MAGIC = 1, FLAC_MBH =2, FLAC_SINFO = 3, FLAC_PADDING = 4, FLAC_APP = 5,
                 FLAC_SEEK = 6, FLAC_VORBIS = 7, FLAC_CUESHEET = 8, FLAC_PICTURE = 9, FLAC_OKAY = 100};
    enum : int { M4A_BEGIN = 0, M4A_FTYP = 1, M4A_CHK = 2, M4A_MOOV = 3, M4A_FREE = 4, M4A_TRAK = 5, M4A_MDAT = 6,
                 M4A_ILST = 7, M4A_MP4A = 8, M4A_AMRDY = 99, M4A_OKAY = 100};
    enum : int { OGG_BEGIN = 0, OGG_MAGIC = 1, OGG_HEADER = 2, OGG_FIRST = 3, OGG_AMRDY = 99, OGG_OKAY = 100};

    const char volumetable[22]={   0,50,60,65,70,75,80,82,84,86,
                                  88,90,91,92,93,94,95,96,97,98,99,100}; //22 elements

    File              audiofile;    // @suppress("Abstract class cannot be instantiated")
    WiFiClient        client;       // @suppress("Abstract class cannot be instantiated")
    WiFiClientSecure  clientsecure; // @suppress("Abstract class cannot be instantiated")
    WiFiUDP           udpclient;    // @suppress("Abstract class cannot be instantiated")
    uint8_t           cs_pin ;                        	// Pin where CS line is connected
    uint8_t           dcs_pin ;                       	// Pin where DCS line is connected
    uint8_t           dreq_pin ;                      	// Pin where DREQ line is connected
    uint8_t           curvol ; 


    const uint8_t vs1053_chunk_size = 32 ;
    // SCI Register
    const uint8_t SCI_MODE          = 0x0 ;
    const uint8_t SCI_STATUS        = 0x1 ;
    const uint8_t SCI_BASS          = 0x2 ;
    const uint8_t SCI_CLOCKF        = 0x3 ;
    const uint8_t SCI_DECODE_TIME   = 0x4 ;
    const uint8_t SCI_AUDATA        = 0x5 ;
    const uint8_t SCI_WRAM          = 0x6 ;
    const uint8_t SCI_WRAMADDR      = 0x7 ;
    const uint8_t SCI_HDAT0         = 0x8 ;
    const uint8_t SCI_HDAT1         = 0x9 ;
    const uint8_t SCI_AIADDR        = 0xA ;
    const uint8_t SCI_VOL           = 0xB ;
    const uint8_t SCI_AICTRL0       = 0xC ;
    const uint8_t SCI_AICTRL1       = 0xD ;
    const uint8_t SCI_AICTRL2       = 0xE ;
    const uint8_t SCI_AICTRL3       = 0xF ;
    const uint16_t WRAM_VERSION     = 0x1E02;
    const uint16_t WRAM_CONFIG1     = 0x1E03;
    const uint16_t WRAM_PLAYSPEED   = 0x1E04;
    const uint16_t WRAM_BYTERATE    = 0x1E05;
    const uint16_t WRAM_ENDFILLBYTE = 0x1E06;
    const uint16_t WRAM_JUMPPOINTS  = 0x1E16;
    const uint16_t WRAM_LATESTJUMP  = 0x1E26;
    const uint16_t WRAM_POSMSEC     = 0x1E27;
    const uint16_t WRAM_RSYNC       = 0x1E29;
    // SCI_MODE bits
    const uint8_t SM_SDINEW         = 11 ;        	// Bitnumber in SCI_MODE always on
    const uint8_t SM_RESET          = 2 ;        	// Bitnumber in SCI_MODE soft reset
    const uint8_t SM_CANCEL         = 3 ;         	// Bitnumber in SCI_MODE cancel song
    const uint8_t SM_TESTS          = 5 ;         	// Bitnumber in SCI_MODE for tests
    const uint8_t SM_LINE1          = 14 ;        	// Bitnumber in SCI_MODE for Line input

    SPISettings     VS1053_SPISettings;                     // SPI settings for this slave
    SPIClass        *SPIbus         = 0;
    fs::FS          *FileSystem     = 0;

    char            chbuf[512];
    char            m_lastHost[256];                // Store the last URL to a webstream
//    char*           m_playlistBuff = NULL;          // stores playlistdata
//    const uint16_t  m_plsBuffEntryLen = 256;        // length of each entry in playlistBuff
//    filter_t        m_filter[3];                    // digital filters
    int             m_LFcount = 0;                      // Detection of end of header
//    uint32_t        m_sampleRate=16000;
    uint16_t        m_bitrate = 0;                  // Bitrate in kb/sec
    uint32_t        m_avr_bitrate = 0;              // average bitrate, median computed by VBR
//    int             m_readbytes=0;                  // bytes read
//    int             m_metalen=0;                    // Number of bytes in metadata
    int             m_controlCounter = 0;           // Status within readID3data() and readWaveHeader()
//    int8_t          m_balance = 0;                  // -16 (mute left) ... +16 (mute right)
    uint8_t         m_vol=64;                       // volume
 
 
    uint8_t         m_codec = CODEC_NONE;           //
    uint8_t         m_rev=0;                        // Revision
    uint8_t         m_playlistFormat = 0;           // M3U, PLS, ASX
    size_t          m_file_size = 0;                // size of the file
    size_t          m_audioDataSize = 0;            //
    uint32_t        m_audioDataStart = 0;           // in bytes
    int             m_id3Size=0;                    // length id3 tag
    bool            m_f_ssl=false;
    uint8_t         m_endFillByte ;                 // Byte to send when stopping song
    uint16_t        m_datamode=0;                   // Statemaschine
    bool            m_f_chunked = false ;           // Station provides chunked transfer
    bool            m_f_ctseen=false;               // First line of header seen or not
    bool            m_f_firstchunk=true;            // First chunk as input
    bool            m_f_swm = true;                 // Stream without metadata
    bool            m_f_tts = false;                // text to speech
    bool            m_f_webfile = false;
    bool            m_f_firstCall = false;          // InitSequence for processWebstream and processLokalFile
    uint32_t        m_chunkcount = 0 ;              // Counter for chunked transfer
    uint32_t        m_contentlength = 0;
    uint32_t        m_metaint = 0;                  // Number of databytes between metadata
    uint32_t        m_t0 = 0;                       // store millis(), is needed for a small delay
    int16_t         m_btp=0;                        // Bytes to play
    int             m_metacount=0;                  // Number of bytes in metadata
    bool            m_firstmetabyte=false;          // True if first metabyte (counter)
    bool            m_f_running = false;
    bool            m_f_localfile = false ;         // Play from local mp3-file
    bool            m_f_webstream = false ;         // Play from URL
    bool            m_f_ogg=false;                  // Set if oggstream
    bool            m_f_stream_ready=false;         // Set after connecttohost and first streamdata are available
    bool            m_f_unsync = false;
    bool            m_f_exthdr = false; 
    bool            m_f_loop = false;               // Set if audio file should loop
    uint32_t        m_PlayingStartTime = 0;         // Stores the milliseconds after the start of the audio
    bool            m_f_playing = false;            // valid mp3 stream recognized
    uint32_t        m_f_position = 0;
    uint16_t        m_jumpToTime = 0;
    uint32_t        m_jumpToPos = 0;

};
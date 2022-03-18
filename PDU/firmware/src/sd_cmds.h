/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _SD_CARD_CMD_H    /* Guard against multiple inclusion */
#define _SD_CARD_CMD_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

  // Definitions for MMC/SD CARD command
  #define CMD0   (64 + 0)                    // Software Reset Command
  #define CMD1   (64 + 1)                    // Initiate initialization process.
  #define ACMD41 (64 + 41)                   // SEND_OP_COND (SDC)
  #define CMD8   (64 + 8)                    // COMMAND  8  VERSION CHECK  For only SDC V2. Check voltage range.
  #define CMD9   (64 + 9)                    // SEND_CSD  Read CSD register.
  #define CMD10  (64 + 10)                   // SEND_CID  Read CID register.
  #define CMD12  (64 + 12)                   // STOP_TRANSMISSION  Stop to read data.
  #define ACMD13 (64 + 13)                   // SD_STATUS (SDC)
  #define CMD16  (64 + 16)                   // SET SECTOR LENGTH OF BYTES YOU WANT TO READ Change R/W block size.
  #define CMD17  (64 + 17)                   // READ SINGLE SECTOR
  #define CMD18  (64 + 18)                   // READ MULTIPLE SECTORS
  #define CMD23  (64 + 23)                   // SET SECTOR COUNT For only MMC.
  #define ACMD23 (64 + 23)                   // SET_WRITE SECTOR ERASE COUNT (SDC) For only SDC. Define number of blocks to pre-erase with next multi-block write command.
  #define CMD24  (64 + 24)                   // SET WRITE ADDRESS FOR SINGLE SECTOR
  #define CMD25  (64 + 25)                   // SET FIRST WRITE ADDRESS FOR MULTIPLE SECTORS
  #define CMD41  (64 + 41)                   // SEND_OP_COND (ACMD)
  #define CMD55  (64 + 55)                   // APP_CMD  Leading command before ACMD command.
  #define CMD58  (64 + 58)                   // READ_OCR

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */

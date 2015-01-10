/****************************************************************************
 * config/omega/src/nsh.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdlib.h>
#include <nuttx/fs/fixfs.h>

/****************************************************************************
 * File Definitions
 ****************************************************************************/

#define STATE_FILE         CONFIG_METERING_METER_STATE_FILE
#define STATE_FILE_OFFSET  (0)
#define STATE_FILE_SIZE    (48 * 4*1024)

#define READS_FILE         CONFIG_METERING_METER_READS_FILE
#define READS_FILE_OFFSET  (STATE_FILE_OFFSET + STATE_FILE_SIZE)
#define READS_FILE_SIZE    (3*12*31 * 1024) // 3 years of 1KiB daily reads

#define CONFIG_FILE        CONFIG_METERING_METER_CONFIG_FILE
#define CONFIG_FILE_OFFSET (READS_FILE_OFFSET + READS_FILE_SIZE)
#define CONFIG_FILE_SIZE   (2*4*1024)

#define LPSET1_FILE        CONFIG_METERING_METER_LPSET1_FILE
#define LPSET1_FILE_OFFSET (CONFIG_FILE_OFFSET + CONFIG_FILE_SIZE)
#define LPSET1_FILE_SIZE   (256*1024)

#define LPSET2_FILE        CONFIG_METERING_METER_LPSET2_FILE
#define LPSET2_FILE_OFFSET (LPSET1_FILE_OFFSET + LPSET1_FILE_SIZE)
#define LPSET2_FILE_SIZE   (1536*1024)

#define LOG_FILE           CONFIG_METERING_METER_LOG_FILE
#define LOG_FILE_OFFSET    (LPSET2_FILE_OFFSET + LPSET2_FILE_SIZE)
#define LOG_FILE_SIZE      (48 * 4*1024)

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct fixfs_file fixfs_files[] = {
	{STATE_FILE,  STATE_FILE_OFFSET,  STATE_FILE_SIZE},
	{READS_FILE,  READS_FILE_OFFSET,  READS_FILE_SIZE},
	{CONFIG_FILE, CONFIG_FILE_OFFSET, CONFIG_FILE_SIZE},
	{LPSET1_FILE, LPSET1_FILE_OFFSET, LPSET1_FILE_SIZE},
	{LPSET2_FILE, LPSET2_FILE_OFFSET, LPSET2_FILE_SIZE},
	{LOG_FILE,    LOG_FILE_OFFSET,    LOG_FILE_SIZE},
	{NULL, 0, 0}
};
#include "main.h"
#include "emfat.h"
#include "stdlib.h"

#define README_SIZE  	256		//including string length itoa "Version_MSD_2"
#define INDEX_SIZE		256
#define TEST_SIZE		868
/*
const char *autorun_file =
	"[autorun]\r\n"
	"label=MSD_2\r\n";
	//"icon=icon.ico\r\n";
*/

const char readme_file[] =
	"Open file \"index.htm\", an internet connection is required\r\n\r\n"
	"Firmware version ";

const char index_file [] =
	"<html><head><meta http-equiv=\"refresh\" content=\"0; url=https://plutongroup.github.io/msd_2/\"/><title>MSD_2 Shortcut</title></head><body></body></html>";

const char test_file[TEST_SIZE] = {
	0x52, 0x61, 0x72, 0x21, 0x1A, 0x07, 0x00, 0xCF, 0x90, 0x73, 0x00, 0x00, 0x0D, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x40, 0x13, 0x74, 0x20, 0x90, 0x32, 0x00, 0x17, 0x03, 0x00, 0x00, 0x5C,
	0x61, 0x00, 0x00, 0x02, 0x1B, 0xB4, 0x5C, 0xBD, 0xFC, 0x8B, 0x7C, 0x56, 0x1D, 0x33, 0x0D, 0x00,
	0x20, 0x00, 0x00, 0x00, 0x32, 0x5F, 0x31, 0x38, 0x5F, 0x74, 0x65, 0x73, 0x74, 0x2E, 0x6D, 0x62,
	0x70, 0x00, 0xB0, 0x62, 0x15, 0x7B, 0x0C, 0x1D, 0x51, 0x0C, 0xCC, 0xBD, 0x1C, 0x11, 0xB2, 0x49,
	0x64, 0x95, 0x5F, 0xE1, 0x5E, 0x05, 0x04, 0x84, 0x24, 0x23, 0x81, 0xC0, 0xE0, 0x08, 0x56, 0x96,
	0x9F, 0xC2, 0x47, 0x0A, 0xA9, 0x78, 0x07, 0x02, 0x89, 0x52, 0x80, 0x83, 0x8D, 0x56, 0xA9, 0x68,
	0x5A, 0x49, 0x6C, 0x52, 0xCB, 0x0A, 0x1D, 0x8E, 0x17, 0xA4, 0xF8, 0x71, 0xCA, 0xF4, 0xBD, 0x04,
	0x8E, 0x22, 0x39, 0x1C, 0x0E, 0x21, 0xCA, 0xFC, 0x00, 0x48, 0xE6, 0x08, 0xE3, 0x58, 0xBD, 0x9A,
	0xE5, 0xB1, 0x29, 0x0C, 0xDD, 0x72, 0x58, 0xE5, 0x85, 0xE0, 0x70, 0xA9, 0x64, 0x4D, 0xB5, 0xAF,
	0x5A, 0xD7, 0xB2, 0x67, 0xE7, 0xAF, 0x73, 0xF8, 0x6F, 0x5E, 0xA7, 0xF1, 0x4C, 0x58, 0xFC, 0xBF,
	0x3F, 0x3C, 0x7E, 0x5E, 0x5E, 0xFC, 0xF5, 0x6E, 0x3D, 0x6D, 0x6E, 0xA4, 0xF7, 0x12, 0x69, 0x7B,
	0x56, 0x66, 0xF4, 0x48, 0xB6, 0x77, 0xE2, 0x90, 0x50, 0x69, 0x92, 0x47, 0x5F, 0x3A, 0xD3, 0xDC,
	0x64, 0x99, 0x9D, 0xED, 0xE7, 0xA4, 0x52, 0x2D, 0x12, 0xA0, 0x5E, 0xE3, 0x3F, 0x4F, 0x1A, 0x87,
	0xC5, 0xA7, 0x19, 0xD3, 0x3B, 0x6F, 0x85, 0x2A, 0xBC, 0xE5, 0xAC, 0xC3, 0xA0, 0x5E, 0x22, 0xEF,
	0x32, 0xDB, 0x5C, 0x4F, 0xE9, 0xDD, 0x9D, 0xE6, 0x74, 0xFF, 0x4F, 0x94, 0xBC, 0x88, 0xD0, 0x51,
	0x43, 0xA9, 0xFF, 0xA2, 0xE3, 0xD1, 0x2C, 0x45, 0xA0, 0x98, 0xA3, 0xAC, 0x31, 0x86, 0x38, 0xEB,
	0x8E, 0xC0, 0xEC, 0x8E, 0xD1, 0xB9, 0x48, 0xD3, 0x2C, 0xE2, 0xD4, 0x63, 0xBE, 0xB9, 0xB9, 0x60,
	0xF7, 0xCB, 0xB8, 0x88, 0xEA, 0xEE, 0xD1, 0x44, 0xA9, 0xC5, 0x3D, 0xCF, 0x3D, 0xAB, 0x76, 0xD2,
	0x95, 0x62, 0x52, 0xDC, 0x95, 0x53, 0x9E, 0x5B, 0x2F, 0xBC, 0x48, 0xA0, 0x66, 0xBC, 0x5E, 0x16,
	0x40, 0xFF, 0xD8, 0xF9, 0x48, 0x81, 0x66, 0x27, 0x80, 0xDC, 0xAF, 0xBD, 0x48, 0xC3, 0x28, 0x16,
	0xAB, 0x99, 0xEB, 0x7F, 0xD0, 0x2D, 0x34, 0x9D, 0x60, 0x45, 0xED, 0x79, 0x66, 0xAF, 0x4B, 0x64,
	0xE5, 0xA0, 0xC5, 0xB3, 0xDC, 0x67, 0x7A, 0xE6, 0xEF, 0x3D, 0x6B, 0xDD, 0x4A, 0x43, 0x44, 0x59,
	0x0B, 0x92, 0xD6, 0x29, 0x9C, 0x89, 0xDD, 0x55, 0x88, 0xC9, 0x47, 0x1A, 0x49, 0xAA, 0xC6, 0xCE,
	0x40, 0xEC, 0x34, 0x56, 0x6C, 0x4B, 0x2A, 0xC8, 0xCE, 0x0E, 0x92, 0xC8, 0x6C, 0xE4, 0x45, 0x98,
	0xC1, 0xAC, 0x51, 0x8E, 0x32, 0xCF, 0x0E, 0x07, 0x5A, 0x2F, 0x75, 0xB0, 0xE4, 0xDE, 0xC0, 0xB7,
	0x5B, 0x69, 0xB3, 0x52, 0x02, 0x3D, 0x71, 0x7A, 0x28, 0xE2, 0x3D, 0xA8, 0x50, 0xAF, 0x4D, 0x44,
	0x94, 0x9D, 0x05, 0xD7, 0xDA, 0xED, 0xA1, 0x5E, 0x5D, 0x16, 0xDF, 0x36, 0xF0, 0xF1, 0x2C, 0x44,
	0x18, 0xD5, 0x3C, 0xD5, 0x4D, 0xAB, 0x52, 0x31, 0x4F, 0x80, 0x5C, 0x29, 0xAF, 0x41, 0x93, 0x5D,
	0x79, 0xCA, 0x45, 0xD5, 0x4D, 0xF6, 0xF8, 0x3B, 0x5F, 0x71, 0x94, 0xD8, 0xD3, 0xB4, 0x91, 0xDE,
	0xE6, 0xAC, 0xD4, 0xF1, 0xC3, 0x68, 0x99, 0x39, 0x12, 0x98, 0xE3, 0x2F, 0xD2, 0x76, 0xA0, 0x9D,
	0x6C, 0x5A, 0x29, 0x6C, 0x9D, 0x3B, 0x10, 0x1B, 0x59, 0xB4, 0x71, 0xA3, 0x3D, 0x8E, 0x80, 0xFB,
	0x5B, 0x85, 0xB7, 0x2A, 0x38, 0xE0, 0x5C, 0xBC, 0x12, 0xDB, 0xBC, 0xB7, 0x8C, 0x2E, 0x59, 0xB4,
	0x86, 0xB4, 0x95, 0xD0, 0x5A, 0xEF, 0xF8, 0x21, 0x65, 0x69, 0x6F, 0xF3, 0x11, 0x29, 0x8B, 0x19,
	0x30, 0x98, 0xC7, 0x7C, 0xD0, 0x44, 0xCD, 0x84, 0x5F, 0x8B, 0x1B, 0xD2, 0x86, 0x4D, 0x79, 0x1C,
	0xCB, 0xFB, 0x72, 0xEC, 0xF4, 0x29, 0x31, 0x67, 0xE1, 0x47, 0x83, 0x6F, 0xD7, 0xB3, 0xDC, 0xC0,
	0x6D, 0xF2, 0x33, 0x8B, 0xDD, 0x9E, 0x88, 0xF6, 0xD6, 0xE2, 0xF0, 0x32, 0xCB, 0x35, 0x30, 0xC6,
	0x67, 0x16, 0x63, 0xA3, 0xAC, 0xF1, 0x94, 0x0C, 0x97, 0xA9, 0x75, 0x05, 0xCB, 0x2B, 0x53, 0x08,
	0xF0, 0xDF, 0xF2, 0x91, 0x4F, 0x69, 0x30, 0x99, 0x63, 0x12, 0x4D, 0x4B, 0xAE, 0x3C, 0x59, 0xBA,
	0x63, 0x23, 0x27, 0x86, 0xE1, 0x36, 0x2C, 0x47, 0xA7, 0xE5, 0x69, 0x7B, 0x79, 0xA7, 0x8A, 0xD0,
	0xBB, 0x96, 0x71, 0x5B, 0xBA, 0x2E, 0x6F, 0x5A, 0x2E, 0x32, 0xD6, 0x3A, 0x99, 0xE9, 0x43, 0x5F,
	0xBA, 0x0B, 0x33, 0x98, 0x67, 0xE2, 0xB6, 0xF3, 0xDF, 0xE2, 0xB9, 0x96, 0xAE, 0x4A, 0x17, 0xA1,
	0x8B, 0x2D, 0x0F, 0x2A, 0xDD, 0x03, 0xBD, 0x9E, 0x51, 0x60, 0x76, 0x13, 0xB9, 0x72, 0x48, 0xBF,
	0xAD, 0x61, 0xA7, 0xF1, 0x75, 0x6F, 0x0F, 0x9C, 0x50, 0x4E, 0xFF, 0xC2, 0x63, 0x08, 0xF3, 0xD8,
	0x9E, 0x8C, 0x1F, 0x5E, 0xD2, 0xE7, 0x96, 0xDD, 0xE3, 0x50, 0xBE, 0x25, 0xD2, 0x33, 0xA8, 0xE2,
	0xF6, 0x49, 0x59, 0x49, 0xB5, 0xCA, 0x35, 0xF7, 0x49, 0x53, 0xC2, 0xE6, 0x97, 0xF5, 0x78, 0xD7,
	0x05, 0xDB, 0xAF, 0x7E, 0x32, 0x2A, 0x7A, 0xF2, 0xDB, 0x1E, 0xB2, 0x22, 0xF5, 0x40, 0x0E, 0xB3,
	0x73, 0xA4, 0x5E, 0xA8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0xB7, 0x99, 0x5F, 0x3B, 0xEF, 0x3C,
	0x51, 0x7A, 0x40, 0x00, 0x01, 0xD3, 0x22, 0xF4, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x03, 0xDE, 0x99, 0x06, 0x52, 0x7F, 0x85, 0xE8, 0x3D, 0xE9, 0x99, 0x1E, 0xD7, 0xC0, 0x5B, 0xE4,
	0x4D, 0xC4, 0xCC, 0x86, 0x7D, 0x92, 0x5D, 0xB4, 0xE0, 0x27, 0xA1, 0x3E, 0x7E, 0xEF, 0x0D, 0x7C,
	0x23, 0xDA, 0xFC, 0x5E, 0x99, 0x86, 0x36, 0xD2, 0x9E, 0x9A, 0x4B, 0x72, 0x4C, 0x19, 0x4F, 0xE4,
	0x9F, 0x70, 0x53, 0x94, 0x6B, 0xDE, 0x66, 0xE9, 0x3F, 0xA3, 0xE6, 0x73, 0x61, 0xE9, 0x7C, 0xC2,
	0x5D, 0x4B, 0xB5, 0x3F, 0x54, 0xD4, 0xDE, 0x70, 0x8F, 0x2B, 0x3A, 0xFE, 0x20, 0xC4, 0x3D, 0x7B,
	0x00, 0x40, 0x07, 0x00
};

// экземпляр виртуальной файловой системы FAT32
emfat_t emfat;

// callback функции чтения данных
//void autorun_read_proc(uint8_t *dest, int size, uint32_t offset, size_t userdata);
//void icon_read_proc(uint8_t *dest, int size, uint32_t offset, size_t userdata);
void readme_read_proc(uint8_t *dest, int size, uint32_t offset, size_t userdata);
void index_read_proc(uint8_t *dest, int size, uint32_t offset, size_t userdata);
void test_read_proc(uint8_t *dest, int size, uint32_t offset, size_t userdata);
void drivers_read_proc(uint8_t *dest, int size, uint32_t offset, size_t userdata);

#define CMA_TIME EMFAT_ENCODE_CMA_TIME(2,4,2019, 13,0,0)
#define CMA { CMA_TIME, CMA_TIME, CMA_TIME }

// Структура ФС
 volatile emfat_entry_t entries[] =
{
	// name          dir    lvl offset  size             max_size        user  time  read               write
	{ "",            true,  0,  0,      0,               0,              0,    CMA,  NULL,              NULL }, // root
	{ "index.htm",   false, 1,  0,      INDEX_SIZE,      1024*1024,  	 0,    CMA,  index_read_proc,   NULL }, // index.html
	{ "readme.txt",  false, 1,  0,      README_SIZE,     README_SIZE,    0,    CMA,  readme_read_proc,  NULL }, // readme.txt
	{ "test.rar",  	 false, 1,  0,      TEST_SIZE,   	 TEST_SIZE,      0,    CMA,  test_read_proc, 	NULL }, // test.rar
	{ NULL }
};

/*
static emfat_entry_t entries[] =
{
	// name          dir    lvl offset  size             max_size        user  read               write
	{ "",            true,  0,  0,      0,               0,              0,    NULL,              NULL }, // root
	{ "autorun.inf", false, 1,  0,      AUTORUN_SIZE,    1*1024*1024*1024,   0,    autorun_read_proc, NULL }, // autorun.inf
	{ "icon.ico",    false, 1,  0,      ICON_SIZE,       1*1024*1024*1024,      0,    icon_read_proc,    NULL }, // icon.ico
	{ "drivers",     true,  1,  0,      0,               0,              0,    NULL,              NULL }, // drivers/
	{ "readme.txt",  false, 2,  0,      README_SIZE,     1*1024*1024*1024,    0,    readme_read_proc,  NULL }, // drivers/readme.txt
	{ "abc.txt",  false, 2,  0,      README_SIZE,     1*1024*1024*1024-32768+8192,    0,    readme_read_proc,  NULL }, // drivers/readme.txt
	{ NULL }
};
*/
/*
void autorun_read_proc(uint8_t *dest, int size, uint32_t offset, size_t userdata)
{
	int len = 0;
	if (offset > AUTORUN_SIZE) return;
	if (offset + size > AUTORUN_SIZE)
		len = AUTORUN_SIZE - offset; else
		len = size;
	memcpy(dest, &autorun_file[offset], len);
}
*/
/*
void icon_read_proc(uint8_t *dest, int size, uint32_t offset, size_t userdata)
{
	int len = 0;
	if (offset > ICON_SIZE) return;
	if (offset + size > ICON_SIZE)
		len = ICON_SIZE - offset; else
		len = size;
	memcpy(dest, &icon_file[offset], len);
}
*/
 void test_read_proc(uint8_t *dest, int size, uint32_t offset, size_t userdata)
 {
 	int len = 0;
 	if (offset > TEST_SIZE) return;
 	if (offset + size > TEST_SIZE)
 		len = TEST_SIZE - offset; else
 		len = size;
 	memcpy(dest, &test_file[offset], len);
 }

void readme_read_proc(uint8_t *dest, int size, uint32_t offset, size_t userdata)
{
	int32_t end=0, cur, len = 0;
	int32_t Len_String;
	if (offset > README_SIZE) return;
	if (offset + size > README_SIZE)
		len = README_SIZE - offset; else
		len = size;
	char ver_s[8];
	itoa (Version_MSD_2, ver_s, 10);
	end = strlen(&ver_s[0]);
	Len_String = strlen(&readme_file[0]);
	for (uint32_t i = 0; i < len; i++)
	{
		if ((offset+i) < Len_String)
		{
            dest[i]	= readme_file[offset+i];
		}
		else
		{
            cur = (offset+i) - Len_String;
			if (cur<(1+end))
			{
				if(cur==(1))
				{
                    dest[i]	= '.';
				}
				else if(cur>(1))
				{
                    dest[i]	= ver_s[cur-1];
				}
				else
				{
                    dest[i]	= ver_s[cur];
				}
			}
			else
			{
                dest[i]='\r';
			}
		}
	}
}

void index_read_proc(uint8_t *dest, int size, uint32_t offset, size_t userdata)
{
	int32_t Len_String;
	int len = 0;
	if (offset > INDEX_SIZE) return;
	if (offset + size > INDEX_SIZE)
		len = INDEX_SIZE - offset; else
		len = size;

	Len_String = strlen(&index_file[0]);
	for (uint32_t i = 0; i < len; i++)
	{
		if ((offset+i) < Len_String)
		{
            dest[i]	= index_file[offset+i];
		}
		else
		{
            dest[i]=' ';
		}
	}
}


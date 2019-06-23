#include <stdio.h>
#include <string.h>

#define DEV_NAME_LEN    256
#define DEFAULT_NAME    "/dev/FM24"
#define FM24_SIZE	(8 * 256)

char device_name[DEV_NAME_LEN];
char buff[FM24_SIZE];

int main(int argc, char **argv)
{
	FILE *fp = NULL;
	char rand = (char)0;
	int i = 0;

        printf("***** eeprom_test *****\n");
 
        if (argc == 2) {
                strncpy(device_name, argv[1], DEV_NAME_LEN);
        } else {
                strncpy(device_name, DEFAULT_NAME, DEV_NAME_LEN);
        }
 
	fp = fopen("/dev/random", "r");
	if (fp == NULL) {
		printf("Can't get random number\n");
		return -1;
	}
	rand = fgetc(fp);

        printf("Device name: %s\n", device_name);
 
	fp = fopen(device_name, "r+");
	if (fp == NULL) {
		printf("Can't open device\n");
		return -2;
	}
        
	printf("Random symbol: %c\n", rand);
	memset(buff, rand, FM24_SIZE);
	printf("Writing...\n");
	if (fwrite(buff, sizeof(char), FM24_SIZE, fp) != FM24_SIZE) {
		printf("Error during writing\n");
		return -3;
	}
	fseek(fp, 0, SEEK_SET);	
	
	memset(buff, rand + 1, FM24_SIZE);
	printf("Reading...\n");
	if (fread(buff, sizeof(char), FM24_SIZE, fp) != FM24_SIZE) {
		printf("Error during reading\n");
		return -4;
	}

	printf("Comparing...\n");
	for (i = 0; i < FM24_SIZE; ++i) {
		if (buff[i] != rand) {
			printf("Mismatch found. Test failed\n");
			return -5;
		}
	}
        
	printf("Test success\n");
        return 0;
}

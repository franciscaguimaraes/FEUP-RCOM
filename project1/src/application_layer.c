// Application layer protocol implementation

#include "../include/application_layer.h"



int createControlPacket(char* filename, int fileSize, int start, unsigned char* packet){
    
    //L1 é so um byte ent V (o filename) n pode passar de 255
    if(strlen(filename) > 255) {
        printf("size of filename can't fit in 1 byte\n");
        return -1;
    }
    unsigned char size_string[20];
    sprintf(size_string, "%02lx", fileSize);
    int sizeBytes = strlen(size_string) / 2;

    if(start) packet[0] = 0x02; //start
    else packet[0] = 0x03; // end

    packet[1] = 0; //tamanho do ficheiro 
    packet[2] = sizeBytes;

    int i = 4;
    for(int j = (sizeBytes - 1); j > -1; j--){
		packet[i] = fileSize >> (j*8);
        i++;
	}

    packet[i] = 1; //nome do ficheiro
    i++; 

    int filename_size = strlen(filename);
    
    packet[i] = filename_size;
    i++;
    for(int j = 0; j < filename_size; j++){
		packet[i] = filename[i];
        i++;
	}

    return i;

}

int createDataPacket(unsigned char* packet, unsigned int nBytes, int index){

    unsigned char buf[500] = {0};

    buf[0] = 0x01; //C
	buf[1] = index%255; //numero de sequencia
    buf[2] = nBytes/256;//L2
    buf[3] = nBytes%256;//L1

    for(int i = 4; i < nBytes; i++){ 
        buf[i] = packet[i];
    }

    for (int j = 0; j < (nBytes+4); j++) {
        packet[j] = buf[j];
    }

	return (nBytes+4); //tamanho do data packet

}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename) {
    printf("in the application layer\n");

    LinkLayer ll;
    strcpy(ll.serialPort, serialPort);
    ll.baudRate = baudRate;
    if(strcmp(role, "rx") == 0){
        ll.role = LlRx;
    }
    else if(strcmp(role,"tx") == 0){
        ll.role = LlTx;
    }
    ll.nRetransmissions = nTries;
    ll.timeout = timeout;

    if (llopen(ll) == -1) {
        printf("\nCouldn't estabilish the connection\n");
        return;
    } else {
        printf("\nConnection estabilished\n");
    }

    
    if(ll.role == LlTx){
        unsigned char packet[500];
        struct stat st;
        stat(filename, &st);
        int file = open(filename, O_RDONLY);

        if(!file){
            printf("Could not open file\n");
            return;
        }
        else {
            printf("Open file successfully\n");
        }

        unsigned int sizePac = createControlPacket(filename, st.st_size, 1, &packet);
        if(llwrite(packet, sizePac) < 0){
            llclose(0, ll);
            return;
        }
        unsigned int bytes;
        unsigned int index = 0;
        int count = 0;
        while ((bytes = read(file, packet, 500-4)) > 0) {
            index++;
            count += bytes;
            bytes = createDataPacket(&packet, bytes, index);
            if (llwrite(packet, bytes) < 0) {
                printf("Failed to send information frame\n");
                llclose(0, ll);
                return;
            }
            printf("Sending: %d/%d (%d%%)\n", count, st.st_size, (int) (((double)count / (double)st.st_size) *100));
        }
        bytes = createControlPacket(filename, st.st_size, 0, &packet); //end
        
        if (llwrite(packet, bytes) < 0) {
            printf("Failed to send information frame\n");
        }
        close(file);
        
    }
    else if (ll.role == LlRx){
        int *file = -1;
        int STOP = FALSE;
        while(!STOP){
            unsigned char packet[600] = {0};
            int sizePacket = 0;
            int response = llread(&packet, &sizePacket);
            if(response < 0){
                continue;
            }
            if(packet[0] == 0x02){ //start control
                printf("\nStart control\n");
                file = fopen(filename, "wb"); 
            }
            else if(packet[0]==0x03){ //end control
                printf("\nEnd control\n");
                fclose(file);
                STOP = TRUE;  
            }
            else{ //data
                for(int i=4; i<sizePacket; i++){
                    fputc(packet[i], file);
                }
            }
        }
        
    }

    if(llclose(1, ll)< 0){
        printf("Couldn't close\n");
    }
    
}

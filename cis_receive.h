#ifndef cis_receive_h
#define cis_receive_h

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES (see http://www.zachburlingame.com/2011/05/resolving-redefinition-errors-betwen-ws2def-h-and-winsock-h/)
//---------------------------------------------------------------------------------------------------------------------------------------------------------

#ifdef WIN_VERSION
#define MAXAPI_USE_MSCRT
#endif

#ifndef CIS_RECEIVE
#define CIS_RECEIVE
#endif

#include "ext_syssock.h"	    // CNMAT crossplatform socket library - should be first to avoid winsock2 redeclaration problem

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINES
//---------------------------------------------------------------------------------------------------------------------------------------------------------

//#define RGBA_BUFFER

#define UDP_HEADER_SIZE                          (1)//uint32
#define UDP_NB_PACKET_PER_LINE                   (6)
#define UDP_NB_BYTES_PER_PIXELS                  (4)
#define UDP_PACKET_SIZE                          (((((CIS_PIXELS_NB) / UDP_NB_PACKET_PER_LINE)) + (UDP_HEADER_SIZE)) * (UDP_NB_BYTES_PER_PIXELS))

#define PORT                                     (55151)    //The port on which to listen for incoming data

#ifdef CIS_400DPI
#define CIS_PIXELS_PER_LINE                      (1152)
#else
#define CIS_PIXELS_PER_LINE                      (576)
#endif

#define CIS_ADC_OUT_LINES                        (3)

#define CIS_PIXELS_NB                            ((CIS_PIXELS_PER_LINE * CIS_ADC_OUT_LINES))

#define DEFAULT_MULTI    "192.168.0.1"
#define DEFAULT_PORT    PORT

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTURE
//---------------------------------------------------------------------------------------------------------------------------------------------------------

typedef enum
{
    STARTUP_INFO_HEADER = 0,
    IMAGE_DATA_HEADER,
    IMU_DATA_HEADER,
    HID_DATA_HEADER,
}CIS_Packet_HeaderTypeDef;

typedef enum
{
    IMAGE_COLOR_R = 0,
    IMAGE_COLOR_G,
    IMAGE_COLOR_B,
}CIS_Packet_ImageColorTypeDef;

// Packet header structure defining the common header for all packet types// Structure for packets containing startup information like version info
struct packet_StartupInfo{
    uint8_t type;                         // Identifies the data type
    uint32_t packet_id;                   // Sequence number, useful for ordering packets
    uint8_t version_info[64];             // Information about the version, and other startup details
};

// Structure for image data packets, including metadata for image fragmentation
struct packet_Image{
    uint8_t type;                         // Identifies the data type
    uint32_t packet_id;                   // Sequence number, useful for ordering packets
    uint32_t line_id;                      // Identifier for the fragment of the image
    uint8_t fragment_id;                      // Identifier for the fragment of the image
    uint8_t total_fragments;              // Total number of fragments for the complete image
    uint16_t fragment_size;               // Size of this particular fragment
    uint8_t imageColor;                   // Pointer to the fragmented image data
    uint8_t imageData[CIS_PIXELS_NB / UDP_NB_PACKET_PER_LINE];               // Pointer to the fragmented image data
};

// Structure for packets containing button state information
struct packet_HID {
    uint8_t type;                         // Identifies the data type
    uint32_t packet_id;                   // Sequence number, useful for ordering packets
    uint8_t button_A;                     // State of the buttons (pressed/released, etc.)
    uint8_t button_B;                     // State of the buttons (pressed/released, etc.)
    uint8_t button_C;                     // State of the buttons (pressed/released, etc.)
};

// Structure for packets containing sensor data (accelerometer and gyroscope)
struct packet_IMU {
    uint8_t type;                         // Identifies the data type
    uint32_t packet_id;                   // Sequence number, useful for ordering packets
    int16_t acc[3];                       // Accelerometer data: x, y, and z axis
    int16_t gyro[3];                      // Gyroscope data: x, y, and z axis
    int16_t integrated_acc[3];            // Accelerometer data: x, y, and z axis
    int16_t integrated_gyro[3];           // Gyroscope data: x, y, and z axis
};

typedef struct _cisReceive {    // defines our object's internal variables for each instance in a patch
	t_object    s_ob;			// object header - ALL objects MUST begin with this...
	t_syssocket fd;				// UDP socket
	t_sysaddr   addr;			// Address structure
	int listening;				// listening state
	t_systhread listener;		// thread
	char* multicast;			// multicast address
	int port;					// UDP reveiving port
    bool line_complete;         // TRUE if receive complete ligne
    void *outlet_R;
    void *outlet_G;
    void *outlet_B;
    uint8_t* image_buffer_R;
    uint8_t* image_buffer_G;
    uint8_t* image_buffer_B;
} t_cisReceive;

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// PROTOTYPES
//---------------------------------------------------------------------------------------------------------------------------------------------------------

void *cisReceive_new(t_symbol*, long, t_atom*);
void cisReceive_free(t_cisReceive*);
void cisReceive_assist(t_cisReceive*, void*, long, long, char*);
void cisReceive_read(t_cisReceive*);

int syssock_set(t_cisReceive*);
int syssock_addmulticast(t_syssocket, char*);
int syssock_dropmulticast(t_syssocket, char*);

#endif /* cis_send_h */

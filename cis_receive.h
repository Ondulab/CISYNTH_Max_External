#ifndef cis_receive_h
#define cis_receive_h

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES (see http://www.zachburlingame.com/2011/05/resolving-redefinition-errors-betwen-ws2def-h-and-winsock-h/)
//---------------------------------------------------------------------------------------------------------------------------------------------------------

#ifdef WIN_VERSION
#define MAXAPI_USE_MSCRT
#include "stdint.h"
#endif

#ifndef CIS_RECEIVE
#define CIS_RECEIVE
#endif

#include "ext_syssock.h"	    // CNMAT crossplatform socket library - should be first to avoid winsock2 redeclaration problem

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINES
//---------------------------------------------------------------------------------------------------------------------------------------------------------

//#define RGBA_BUFFER

#define UDP_NB_PACKET_PER_LINE                   (12)
#define UDP_PACKET_SIZE                          ((CIS_PIXELS_NB) / (UDP_NB_PACKET_PER_LINE))

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
    uint32_t line_id;                     // Line identifier
    uint8_t fragment_id;                  // Fragment position
    uint8_t total_fragments;              // Total number of fragments for the complete image
    uint16_t fragment_size;               // Size of this particular fragment
    uint8_t imageData_R[CIS_PIXELS_NB / UDP_NB_PACKET_PER_LINE];               // Pointer to the fragmented red image data
    uint8_t imageData_G[CIS_PIXELS_NB / UDP_NB_PACKET_PER_LINE];               // Pointer to the fragmented green image data
    uint8_t imageData_B[CIS_PIXELS_NB / UDP_NB_PACKET_PER_LINE];               // Pointer to the fragmented blue image data
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
    float_t acc[3];                       // Accelerometer data: x, y, and z axis
    float_t gyro[3];                      // Gyroscope data: x, y, and z axis
    float_t integrated_acc[3];            // Accelerometer data: x, y, and z axis
    float_t integrated_gyro[3];           // Gyroscope data: x, y, and z axis
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
    
    void *outlet_Image;
    void *outlet_LowImage;
    void *outlet_IMU;
    void *outlet_HID;
    
    uint8_t* image_buffer_R;
    uint8_t* image_buffer_G;
    uint8_t* image_buffer_B;
    t_atom *atom_buffer_R;
    t_atom *atom_buffer_G;
    t_atom *atom_buffer_B;
    
    float_t IMU_Ax;
    float_t IMU_Ay;
    float_t IMU_Az;
    t_atom *atom_IMU_Ax;
    t_atom *atom_IMU_Ay;
    t_atom *atom_IMU_Az;
    
    float_t IMU_Gx;
    float_t IMU_Gy;
    float_t IMU_Gz;
    t_atom *atom_IMU_Gx;
    t_atom *atom_IMU_Gy;
    t_atom *atom_IMU_Gz;
    
    uint8_t HID_B1;
    uint8_t HID_B2;
    uint8_t HID_B3;
    t_atom *atom_HID_B1;
    t_atom *atom_HID_B2;
    t_atom *atom_HID_B3;
    
    t_clock *clock; // Ajout d'un clock pour gérer le timing
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

#ifndef cis_receive_h
#define cis_receive_h

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES (see http://www.zachburlingame.com/2011/05/resolving-redefinition-errors-betwen-ws2def-h-and-winsock-h/)
//---------------------------------------------------------------------------------------------------------------------------------------------------------

#ifdef _WIN32
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <io.h>
#include <stdint.h>
typedef SSIZE_T ssize_t;
#else
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <fcntl.h>
#endif

#ifdef _WIN32
#define PACKED_STRUCT __declspec(align(4))
#else
#define PACKED_STRUCT __attribute__((aligned(4)))
#endif

#ifndef CIS_RECEIVE
#define CIS_RECEIVE
#endif

#include "ext_syssock.h"	    // CNMAT crossplatform socket library - should be first to avoid winsock2 redeclaration problem

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINES
//---------------------------------------------------------------------------------------------------------------------------------------------------------

#define CIS_400DPI_PIXELS_NB                    (3456)
#define CIS_200DPI_PIXELS_NB                    (1728)

#define CIS_MAX_PIXELS_PER_LANE                 (CIS_400DPI_PIXELS_PER_LANE)

// Number of UDP packets per line
#define UDP_MAX_NB_PACKET_PER_LINE              (12)

#define CIS_400DPI_PIXELS_NB                    (3456)
#define CIS_200DPI_PIXELS_NB                    (1728)

#define CIS_MAX_PIXELS_NB                       (CIS_400DPI_PIXELS_NB)

// Ensure UDP_LINE_FRAGMENT_SIZE is an integer
#if (CIS_MAX_PIXELS_NB % UDP_MAX_NB_PACKET_PER_LINE) != 0
  #error "CIS_MAX_PIXELS_NB must be divisible by UDP_NB_PACKET_PER_LINE."
#endif

// Size of each UDP line fragment (number of pixels per packet)
#define UDP_LINE_FRAGMENT_SIZE                  (CIS_MAX_PIXELS_NB / UDP_MAX_NB_PACKET_PER_LINE)

#define PORT                                    (55151)    //The port on which to listen for incoming data

#define DEFAULT_MULTI                           "192.168.0.1"
#define DEFAULT_PORT                            PORT

//---------------------------------------------------------------------------------------------------------------------------------------------------------
//  COMMON STRUCTURE CIS / MAX
//---------------------------------------------------------------------------------------------------------------------------------------------------------

typedef enum
{
    SW1  = 0,
    SW2,
    SW3,
}buttonIdTypeDef;

typedef enum
{
    SWITCH_RELEASED = 0,
    SWITCH_PRESSED
}buttonStateTypeDef;

typedef enum
{
    LED_1 = 0,
    LED_2,
    LED_3,
}ledIdTypeDef;

typedef enum
{
    STARTUP_INFO_HEADER = 0x11,
    IMAGE_DATA_HEADER = 0x12,
    IMU_DATA_HEADER = 0x13,
    BUTTON_DATA_HEADER= 0x14,
    LED_DATA_HEADER = 0x15,
}CIS_Packet_HeaderTypeDef;

typedef enum
{
    IMAGE_COLOR_R = 0,
    IMAGE_COLOR_G,
    IMAGE_COLOR_B,
}CIS_Packet_ImageColorTypeDef;

typedef enum
{
    CIS_CAL_REQUESTED = 0,
    CIS_CAL_START,
    CIS_CAL_PLACE_ON_WHITE,
    CIS_CAL_PLACE_ON_BLACK,
    CIS_CAL_EXTRACT_INNACTIVE_REF,
    CIS_CAL_EXTRACT_EXTREMUMS,
    CIS_CAL_EXTRACT_OFFSETS,
    CIS_CAL_COMPUTE_GAINS,
    CIS_CAL_END,
}CIS_Calibration_StateTypeDef;

// Packet header structure defining the common header for all packet types// Structure for packets containing startup information like version info
struct PACKED_STRUCT packet_StartupInfo
{
    CIS_Packet_HeaderTypeDef type;         // Identifies the data type
    uint32_t packet_id;                   // Sequence number, useful for ordering packets
    uint8_t version_info[32];             // Information about the version, and other startup details
};

// Structure for image data packets, including metadata for image fragmentation
struct PACKED_STRUCT packet_Image
{
    CIS_Packet_HeaderTypeDef type;                     // Identifies the data type
    uint32_t packet_id;                               // Sequence number, useful for ordering packets
    uint32_t line_id;                                  // Line identifier
    uint8_t fragment_id;                              // Fragment position
    uint8_t total_fragments;                          // Total number of fragments for the complete image
    uint16_t fragment_size;                           // Size of this particular fragment
    uint8_t imageData_R[UDP_LINE_FRAGMENT_SIZE];       // Pointer to the fragmented red image data
    uint8_t imageData_G[UDP_LINE_FRAGMENT_SIZE];      // Pointer to the fragmented green image data
    uint8_t imageData_B[UDP_LINE_FRAGMENT_SIZE];    // Pointer to the fragmented blue image data
};

struct PACKED_STRUCT button_State
{
    buttonStateTypeDef state;
    uint32_t pressed_time;
};

// Structure for packets containing button state information
struct PACKED_STRUCT packet_Button
{
    CIS_Packet_HeaderTypeDef type;                         // Identifies the data type
    uint32_t packet_id;                   // Sequence number, useful for ordering packets
    buttonIdTypeDef button_id;                 // Id of the button
    struct button_State button_state;         // State of the led A
};

struct PACKED_STRUCT led_State
{
    uint16_t brightness_1;
    uint16_t time_1;
    uint16_t glide_1;
    uint16_t brightness_2;
    uint16_t time_2;
    uint16_t glide_2;
    uint32_t blink_count;
};

// Structure for packets containing leds state
struct PACKED_STRUCT packet_Leds
{
    CIS_Packet_HeaderTypeDef type;                         // Identifies the data type
    uint32_t packet_id;                   // Sequence number, useful for ordering packets
    ledIdTypeDef led_id;                 // Id of the led
    struct led_State led_state;         // State of the selected led
};

// Structure for packets containing sensor data (accelerometer and gyroscope)
struct PACKED_STRUCT packet_IMU
{
    CIS_Packet_HeaderTypeDef type;                         // Identifies the data type
    uint32_t packet_id;                   // Sequence number, useful for ordering packets
    float_t acc[3];                   // Accelerometer data: x, y, and z axis
    float_t gyro[3];                  // Gyroscope data: x, y, and z axis
    float_t integrated_acc[3];        // Accelerometer data: x, y, and z axis
    float_t integrated_gyro[3];       // Gyroscope data: x, y, and z axis
};

struct PACKED_STRUCT cisRgbBuffers
{
    uint8_t R[CIS_MAX_PIXELS_NB];
    uint8_t G[CIS_MAX_PIXELS_NB];
    uint8_t B[CIS_MAX_PIXELS_NB];
};

//---------------------------------------------------------------------------------------------------------------------------------------------------------
//  MAX STRUCTURE
//---------------------------------------------------------------------------------------------------------------------------------------------------------

typedef struct _cisReceive {    // defines our object's internal variables for each instance in a patch
	t_object    s_ob;			// object header - ALL objects MUST begin with this...
	t_syssocket fd;				// UDP socket
	t_sysaddr   addr;			// Address structure
	int listening;				// listening state
	t_systhread listener;		// thread
	char* multicast;			// multicast address
	int port;					// UDP reveiving port
    
    uint32_t startup_packet_id; // Optional: to store the startup packet ID
    char version_info[64];      	// Added to store version information
    
    void *outlet_sync;
    void *outlet_R;
    void *outlet_G;
    void *outlet_B;
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
    
    struct led_State leds[3];
    long dummy_attr;
    
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

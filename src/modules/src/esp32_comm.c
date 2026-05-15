#include "esp32_comm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart1.h"
#include "system.h"
#include "param.h"
#include "log.h"

// Structs
typedef struct {
    float vectors[28];
    bool is_valid;
} ControllerData;

struct __attribute__((packed)) RequestPacket {
    uint8_t header;
    uint16_t id;
};

struct __attribute__((packed)) ResponsePacket {
    uint8_t header;
    float data[28];
    uint8_t checksum;
};

// Double Buffer variables
static ControllerData buffers[2];
static volatile uint8_t active_buffer_idx = 0; 
static volatile uint16_t requested_controller_id = 0;
static volatile uint16_t current_controller_id = 0;
static float ctrl_val = 0.0f;

// The Background Task
static void esp32CommTask(void* param) {
    // Wait for the system to fully boot before using UART
    systemWaitStart();
    
    uart1Init(2000000);
    
    struct RequestPacket req = { .header = 0xAA, .id = 0 };
    struct ResponsePacket res;

    while(1) {
        uint16_t target_id = requested_controller_id;
        
        if (target_id != current_controller_id && target_id > 0) {
            req.id = target_id;
            
            // Send request (UART1 maps to TX1/RX1 on the deck port)
            uart1SendData(sizeof(req), (uint8_t*)&req);

            int received = 0;
            uint8_t* resPtr = (uint8_t*)&res;
            TickType_t start_time = xTaskGetTickCount();
            
            while (received < sizeof(res)) {
                // Read 1 byte, yield for max 1 tick (~1ms) if the buffer is empty
                if (uart1GetDataWithTimeout(&resPtr[received], 1)) {
                    received++;
                }
                
                // Hard timeout: Drop packet if the whole payload takes more than 10ms
                if ((xTaskGetTickCount() - start_time) > M2T(10)) {
                    break; 
                }
            }

            if (received == sizeof(res) && res.header == 0xBB) {
                // Switch buffers
                uint8_t inactive_idx = 1 - active_buffer_idx;
                for(int i = 0; i < 28; i++) {
                    buffers[inactive_idx].vectors[i] = res.data[i];
                }
                buffers[inactive_idx].is_valid = true;
                
                // ATOMIC SWAP
                active_buffer_idx = inactive_idx; 
                current_controller_id = target_id;
            }
        }
        vTaskDelay(M2T(1)); 

        ctrl_val = buffers[active_buffer_idx].vectors[0];
    }
}

// Initialization function
void esp32CommInit(void) {
    // Initialize buffers to invalid
    buffers[0].is_valid = false;
    buffers[1].is_valid = false;

    // Create the FreeRTOS task. 
    // Priority 3 is standard for background IO (Stabilizer is priority 5 / highest)
    xTaskCreate(esp32CommTask, "esp32Comm", configMINIMAL_STACK_SIZE * 3, NULL, 3, NULL);
}

// API functions
void esp32CommSetControllerId(uint16_t id) {
    requested_controller_id = id;
}

const float* esp32CommGetActiveController(void) {
    uint8_t idx = active_buffer_idx;
    if (buffers[idx].is_valid) {
        return buffers[idx].vectors;
    }
    return NULL;
}



PARAM_GROUP_START(esp)
PARAM_ADD_CORE(PARAM_UINT16, ctrl_id, &requested_controller_id)
PARAM_GROUP_STOP(esp)

/**
 * Logging the ESP params
 */
LOG_GROUP_START(esp)
/**
 * @brief The controller id
 */
LOG_ADD_CORE(LOG_FLOAT, ctrl_val, &ctrl_val)
LOG_GROUP_STOP(esp)
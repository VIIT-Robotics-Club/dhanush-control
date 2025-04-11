#include <stdio.h>
#include "urosHandler.hpp"

#include "qmd.hpp"
#include "ballHandler/ballHandler.hpp"
#include "pinMap.hpp"

#include <esp_log.h>

#include "multiThreadExecutor/workerThread.hpp"

qmd* handler = 0;
urosHandler* uros = 0;


int pwmPins[] = {FLYWL_U_PWM, FLYWL_L_PWM, ARM_MOTOR_PWM, FLYW_ANGLE_L, FLYW_ANGLE_R};
int dirPins[] = {FLYWL_U_DIR, FLYWL_L_DIR, ARM_MOTOR_DIR, G_FLYW_ANGLE_DIR, G_FLYW_ANGLE_DIR};


class testThread : public workerThread {
public:
    testThread(const char* name) : name(name) {};


    void run(){
        while (true){
            queryRunningState();
            ESP_LOGI(name, "hello world ");

            vTaskDelay(pdMS_TO_TICKS(100));
        };
    }

    const char* name = nullptr;
};

extern "C" void app_main(void)
{
    // initialize quad motoer
    handler = new qmd(pwmPins,dirPins,5);
    handler->setRange(19990, 0);
    handler->setRange(2500, 500, 3);
    handler->setRange(500, 2500, 4);

    urosHandler::config cfg = {
        .mode = urosHandler::config::TRANSPORT_USB
    };
    
    uros = new urosHandler(cfg);
    
    // configuration for ball handler
    ball_handler_config_t ball_handle = {
        .qmd_handler = handler,
    };


    // uros->addThreadExecutor({
    //     new ballHandler(ball_handle)     
    // },APP_CPU_NUM);

    testThread t1("wanda"), t2("planetary"), t3("smit");

    t2.start();
    t1.start();
    t3.start();

    while (true)
    {
        ESP_LOGW("main", "blocking task wanda");
        t1.block();
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGW("main", "blocking task planetary");
        t2.block();
        vTaskDelay(pdMS_TO_TICKS(2000));
    
        ESP_LOGW("main", "blocking task smit");
        t3.block();
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        
        ESP_LOGW("main", "freeing task wanda");
        t1.unblock();
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        ESP_LOGW("main", "freeing task planetary");
        t2.unblock();
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        ESP_LOGW("main", "freeing task smit");
        t3.unblock();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
        
}

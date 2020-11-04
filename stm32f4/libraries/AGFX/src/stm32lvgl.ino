#include <lvgl.h>
#include <tft.h>
#include <touchpad.h>

void my_printf(lv_log_level_t level, const char *file, uint32_t line, const char *dsc, const char *dsc1)
{
    Serial3.printf("level:%d,file:%s,line:%d,dsc:%s,%s\r\n",level,file,line,dsc, dsc1);
    Serial3.flush();
}

void setup() {
  
Serial3.begin(115200);
 lv_init();
 tft_init();
 touchpad_init();
 lv_log_register_print_cb(my_printf);

 lv_obj_t *label = lv_label_create(lv_scr_act(), NULL);
 lv_label_set_text(label, "Hello Arduino! (V7.0.X)");
 lv_obj_align(label, NULL, LV_ALIGN_CENTER, 100, 100);

}

void loop() {
  // put your main code here, to run repeatedl:
    lv_tick_inc(5);
    lv_task_handler();
    
    BSP_LED_Toggle(LED3);
    delay(5);

}

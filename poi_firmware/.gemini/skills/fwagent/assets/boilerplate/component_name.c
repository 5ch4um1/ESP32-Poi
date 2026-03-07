#include "component_name.h"
#include "esp_log.h"

static const char *TAG = "COMPONENT_NAME";

esp_err_t component_name_init(void)
{
    ESP_LOGI(TAG, "Initializing component_name...");
    return ESP_OK;
}

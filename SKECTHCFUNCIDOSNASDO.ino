#include <Arduino.h>
#include <SPI.h>
#include <driver/spi_slave.h>
#include <stdio.h>
#include <WiFi.h>
#include <WiFiManager.h> // Include the WiFiManager library
#include <HTTPClient.h>
// API Endpoint
// Replace with your computer's IP address when running the API locally.
// To find your IP, use 'ipconfig' on Windows or 'ifconfig'/'ip a' on macOS/Linux.
const char* api_endpoint = "https://8266-2804-772c-bd3e-0-968c-49d0-5c0d-fef7.ngrok-free.app/lots";
/*
  Este cabeçalho contém as definições para as funções e estruturas de baixo nível
  do driver SPI do ESP32 (ESP-IDF), como:
  - spi_slave_transaction_t
  - spi_slave_interface_config_t
  - spi_bus_config_t
  - spi_slave_initialize()
  - spi_slave_queue_trans()
  - HSPI_HOST / VSPI_HOST
*/


// Pinos padrão para HSPI no ESP32
// MISO -> GPIO 19
// MOSI -> GPIO 23
// SCLK -> GPIO 18
// SS   -> GPIO 5

// Buffer para armazenar os dados recebidos
// A DMA (Direct Memory Access) requer que os buffers estejam em uma memória específica.
// O atributo DMA_ATTR garante isso.
DRAM_ATTR char rx_buf[4096];

// Flag para indicar que uma transação foi completada.
// 'volatile' é importante pois a variável é modificada em uma interrupção.
volatile uint8_t trans_end = 0;

// Função de callback chamada após uma transação SPI ser completada (ISR)
// IRAM_ATTR garante que a função de interrupção seja colocada na RAM para execução rápida.
void IRAM_ATTR post_trans_cb(spi_slave_transaction_t *trans) {
    // Marca o fim da transação para que o loop principal possa processar os dados.
    trans_end = 1;
}

void setup() {
  Serial.begin(115200); // Inicializa a comunicação serial (usei 115200 por ser mais comum)

  // 1. Configuração do barramento SPI (pinos)
  spi_bus_config_t bus_config = {
      .mosi_io_num = 23, // Pino MOSI
      .miso_io_num = 19, // Pino MISO
      .sclk_io_num = 18, // Pino SCLK
      .quadwp_io_num = -1, // Não usado
      .quadhd_io_num = -1, // Não usado
      .max_transfer_sz = 4096 // Tamanho máximo da transferência
  };

  // 2. Configuração da interface do SPI Slave
  spi_slave_interface_config_t slv_int_config = {
      .spics_io_num = 5,      // Pino CS/SS
      .flags = 0,             // Flags (padrão 0)
      .queue_size = 1,        // Tamanho da fila de transações
      .mode = 0,              // Modo SPI (0, 1, 2 ou 3)
      .post_setup_cb = NULL,  // Callback após configuração (não usado)
      .post_trans_cb = post_trans_cb // Callback após transação
  };
  // Connect to WiFi using WiFiManager
  connect_to_wifi();
  // 3. Inicializa o driver do SPI Slave
  // Usando HSPI_HOST. Você ta//mbém poderia usar VSPI_HOST.
  // O último parâmetro é o canal DMA (1 ou 2, ou 0 para desabilitar)
  esp_err_t ret = spi_slave_initialize(VSPI_HOST, &bus_config, &slv_int_config, 1);
  // Verifica se a inicialização ocorreu sem erros
  assert(ret == ESP_OK);

  Serial.println("ESP32 inicializado como Slave SPI. Aguardando dados...");
}

void loop() {
  // Limpa o buffer de recepção antes de cada transação
  //(rx_buf, 0, sizeof(rx_buf));

  // Prepara a estrutura da transação
  spi_slave_transaction_t trans;
  memset(&trans, 0, sizeof(trans));

  // Define os parâmetros da transação
  trans.length = sizeof(rx_buf) * 8; // Comprimento em bits
  trans.tx_buffer = NULL;             // Não estamos enviando dados do slave neste exemplo
  trans.rx_buffer = rx_buf;           // Buffer para receber os dados

  // Enfileira a transação e aguarda o Master iniciar a comunicação
  esp_err_t ret = spi_slave_queue_trans(VSPI_HOST, &trans, portMAX_DELAY);
  assert(ret == ESP_OK);

  // Aguarda a flag da interrupção indicar o fim da transação
  if (trans_end) {
    Serial.print("Dados recebidos: ");
    // Imprime os dados que foram recebidos no buffer
    Serial.print(rx_buf);
    Serial.println();
    
    Serial.printf("Dados em Hex: %x", rx_buf);
    for(int i = 0; i<sizeof(rx_buf); i++){
      Serial.printf("%x", rx_buf[i]);
    }
    Serial.println();

    // HERE: Find the end character and terminate the string
    char *end_of_string = strchr(rx_buf, '~');
    if (end_of_string != NULL) {
        *end_of_string = '\0'; // Replace '~' with null terminator
    }

    // Now rx_buf is a clean, null-terminated JSON string
    Serial.print("Clean JSON: ");
    Serial.println(rx_buf);
    strcpy(payload, rx_buf);
    send_data_to_api(payload);
    // Reseta a flag para a próxima transação
    trans_end = false;
  }
}

/**
 * @brief Handles WiFi connection using the WiFiManager library.
 */
void connect_to_wifi() {
  // Create an instance of the WiFiManager
  WiFiManager wm;

  // Set a timeout for the configuration portal. If no one connects within
  // 3 minutes (180 seconds), the ESP32 will restart.
  wm.setConfigPortalTimeout(180);

  // Try to connect to the last known WiFi network.
  // If it fails, it will start an Access Point with the name "SensorSetupAP".
  // Connect to this AP from your phone or computer to configure the WiFi.
  if (!wm.autoConnect("SensorSetupAP")) {
    Serial.println("[!] Failed to connect and hit timeout. Restarting...");
    delay(3000);
    ESP.restart(); // Restart the ESP
  }

  // If we get here, the connection was successful
  Serial.println("\n[+] WiFi Connected!");
  Serial.print("[+] IP Address: ");
  Serial.println(WiFi.localIP());
}


void send_data_to_api(const char* json_payload) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    Serial.print("[*] Sending POST request to: ");
    Serial.println(api_endpoint);

    // Start the HTTP request
    http.begin(api_endpoint);
    // Set the content type header to indicate JSON data
    http.addHeader("Content-Type", "application/json");

    // Send the POST request with the JSON payload
    int http_response_code = http.POST(json_payload);

    // Check the response
    if (http_response_code > 0) {
      Serial.print("[+] HTTP Response code: ");
      Serial.println(http_response_code);
      String response_body = http.getString();
      Serial.println("[+] Response body:");
      Serial.println(response_body);
    } else {
      Serial.print("[!] HTTP Request failed. Error code: ");
      Serial.println(http_response_code);
    }

    // Free resources
    http.end();
  } else {
    Serial.println("[!] WiFi not connected. Cannot send data.");
  }
}
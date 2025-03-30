#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "ssd1306.h"
#include "doglab.h"
#include "string.h"
#include "pico/cyw43_arch.h"
#include "lwip/opt.h"
#include "lwip/dns.h"
#include "lwip/init.h"
#include "lwip/tcp.h"
#include "lwip/sockets.h"
#include "dht.h"

// --- Configurações de Rede ---
// Credenciais da rede Wi-Fi
#define WIFI_SSID "FRANCIMAR FAUSTINO" // Nome da sua rede Wi-Fi
#define WIFI_PASSWORD "lena1708"       // Senha da sua rede Wi-Fi
#define WIFI_CONNECT_TIMEOUT 30000     // Tempo limite para conexão Wi-Fi (ms)

/// 192.168.15.163/24
// Endereço IP e porta do servidor HTTP
#define IP_OCTET_1 192
#define IP_OCTET_2 168
#define IP_OCTET_3 15
#define IP_OCTET_4 163

#define SERVER_PORT 8000 // Porta do servidor

// --- Configurações de Sensores ---
// Canal ADC para o sensor de temperatura interno
#define TEMP_SENSOR_CHANNEL 4

// Dados do sensor de temperatura e umidade
static const dht_model_t DHT_MODEL = DHT11;
static const uint DHT_PIN = 16;

// Canal ADC e pino para o gás
#define GAS_SENSOR_CHANNEL 2
#define GAS_VREF 1.65f // Tensão de referência do gás (Volts)
#define GAS_ADC_PIN 28

// Identificador do sensor (usado na requisição HTTP)
#define SENSOR_ID 1

// --- Variáveis Globais ---

// Flag para indicar se a conexão com o servidor foi estabelecida
bool server_connected = false;

// --- Estrutura para Armazenar as Leituras dos Sensores ---
typedef struct sensors_read
{
    float temperature; // Temperatura em graus Celsius.
    float gas_level;   // Nível de gás (em porcentagem).
    float moisture;
} SensorsRead;

// Variável global para armazenar as leituras atuais dos sensores.
SensorsRead sensors_reading = {
    temperature : 0.0f,
    gas_level : 0.0f,
    moisture : 0.0f
};

// Variáveis do display (posicionamento na tela)
int x_pos_text = 0;
int y_pos_text = 0;
int SCREEN_BOTTOM_LIMIT = SCREEN_HEIGHT - 8;
char *bottom_screen_text = "Monitoramento";

// Variáveis do sensor de temperatura
dht_t dht;

// --- Protótipos de Funções ---
void clear_display();
void display_text_with_scale(char *text, int scale);
void display_text(char *text);
void draw_screen_botton();
void draw_screen_components();
void read_temperature_and_moisture();
void read_gas_level();
void read_sensors();
void send_data_to_server();
int connect_to_wifi();
err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

/**
 * @brief Limpa o display OLED.
 *
 * Chama a função `ssd1306_clear` para apagar todos os pixels do display e redesenha os componentes
 */
void clear_display()
{
    ssd1306_clear(&display);

    // Reseta variáveis do texto
    x_pos_text = 0;
    y_pos_text = 0;

    draw_screen_components();
}

/**
 * @brief Exibe texto no display OLED com uma escala específica.
 *
 * @param text O texto a ser exibido.
 * @param scale O fator de escala do texto (1 = normal, 2 = dobro, etc.).
 */
void display_text_with_scale(char *text, int scale)
{
    int char_width = scale * 6;
    int text_width = strlen(text) * char_width;
    if (x_pos_text + text_width > SCREEN_WIDTH)
    {
        x_pos_text = 0;
        y_pos_text += 8;

        if (y_pos_text + 8 > SCREEN_BOTTOM_LIMIT)
        {
            y_pos_text = 0;
            clear_display();
        }
    }

    ssd1306_draw_string(&display, x_pos_text, y_pos_text, 1, text);
    ssd1306_show(&display);
    x_pos_text += text_width;
}

/**
 * @brief Exibe texto no display OLED (usando escala 1).
 *  Chama a função `display_text_with_scale` com a escala padrão (1).
 * @param text O texto a ser exibido.
 */
void display_text(char *text)
{
    char msg[strlen(text) + 1];
    sprintf(msg, "%s \n", text);
    display_text_with_scale(text, 1);
}

/**
 * @brief Callback para tratar respostas do servidor TCP.
 *
 * Esta função é chamada pela biblioteca lwIP quando dados são recebidos do servidor.
 * Neste código, ela não processa ativamente a resposta, mas é necessária para
 * a comunicação TCP.
 *
 * @param arg Argumento passado para a função de callback (não usado).
 * @param tpcb Ponteiro para o bloco de controle do protocolo TCP (PCB).
 * @param p Ponteiro para o buffer de dados recebidos (pbuf).
 * @param err Código de erro.
 * @return err_t Retorna ERR_OK.
 */
err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (p == NULL)
    {
        // O servidor fechou a conexão
        tcp_close(tpcb);
        return ERR_OK;
    }
    else
    {
        // Recepção de dados pelo servidor
        tcp_recved(tpcb, p->tot_len);
        pbuf_free(p);
        return ERR_OK;
    }
}

/**
 * @brief Lê a temperatura do DHT11.
 *
 * Seleciona o canal ADC do sensor, lê o valor bruto, converte para voltagem
 * e, em seguida, para temperatura em graus Celsius usando a fórmula do datasheet.
 * Armazena o resultado nas variáveis globais `sensors_reading.temperature` e `sensors_reading.moisture`.
 */
void read_temperature_and_moisture()
{
    dht_start_measurement(&dht);

    float humidity;
    float temperature_c;
    dht_result_t result = dht_finish_measurement_blocking(&dht, &humidity, &temperature_c);
    sensors_reading.temperature = temperature_c;
    sensors_reading.moisture = humidity;

    if (!result == DHT_RESULT_OK)
    {
        display_text("Erro de conexão com o sensor DHT11!");
    }
}

void read_gas_level()
{
    adc_select_input(GAS_SENSOR_CHANNEL); // Seleciona o canal do gás

    float sum = 0;
    const float conversion_f = 3.3f / (1 << 12); // Conversão do ADC para tensão

    // Faz 10 leituras e soma os resultados
    for (int i = 0; i < 10; i++)
    {
        float voltage = adc_read() * conversion_f;
        sum += voltage;
        sleep_ms(10); // Pequeno atraso entre as leituras
    }

    // Calcula a média da tensão lida
    float avg_voltage = sum / 10.0f;

    // Definição dos valores mínimo e máximo esperados do sensor
    const float V_MIN = 0.1f; // Tensão mínima esperada
    const float V_MAX = 3.0f; // Tensão máxima esperada

    // Mapeia a média de tensão para um percentual (0-100%)
    float percentage = ((avg_voltage - V_MIN) / (V_MAX - V_MIN)) * 100.0f;

    // Garante que o valor fique dentro do intervalo 0-100%
    if (percentage < 0.0f)
        percentage = 0.0f;
    if (percentage > 100.0f)
        percentage = 100.0f;

    sensors_reading.gas_level = percentage; // Armazena o resultado como percentual
}

/**
 * @brief Lê ambos os sensores (temperatura e nível de gás).
 *  Chama as funções `read_temperature` e `read_gas_level`.
 */
void read_sensors()
{
    read_temperature_and_moisture();
    read_gas_level();
}

void display_sensor_data()
{
    char msg[255];
    display_text("Monitoriamento Ativo!");
    snprintf(msg, sizeof(msg), "Temperatura: %.2f oC", sensors_reading.temperature);
    display_text(msg);
    snprintf(msg, sizeof(msg), "Umidade: %.2f %%", sensors_reading.moisture);
    display_text(msg);
    snprintf(msg, sizeof(msg), "Nivel de Gas: %.2f", sensors_reading.gas_level);
    display_text(msg);
}

struct tcp_pcb *create_tcp_struct()
{
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb)
    {
        display_text("Erro ao criar PCB!");
        return NULL;
    }
    return pcb;
}

/**
 * @brief Envia os dados dos sensores para o servidor HTTP.
 *
 * Cria uma conexão TCP com o servidor, formata uma requisição HTTP GET
 * com os dados de temperatura e ruído e envia a requisição.
 */
void send_data_to_server(struct tcp_pcb *pcb)
{
    // Define o endereço IP do servidor
    ip_addr_t server_ip;
    IP4_ADDR(&server_ip, IP_OCTET_1, IP_OCTET_2, IP_OCTET_3, IP_OCTET_4);

    // Tenta conectar ao servidor
    err_t connect_err = tcp_connect(pcb, &server_ip, SERVER_PORT, NULL);
    if (connect_err != ERR_OK)
    {
        display_text("Erro ao conectar");
        printf("Erro ao conectar: %d\n", connect_err); // Imprime o código de erro
        tcp_abort(pcb);                                // Aborta a conexão
        tcp_close(pcb);                                // Fecha o PCB
        return;
    }

    // Verifica se a conexão foi estabelecida
    if (!server_connected)
    {
        display_text("O servidor está disponível");
        server_connected = true; // Atualiza o status da conexão
    }

    // Formata a requisição HTTP GET
    char request[512];
    snprintf(request, sizeof(request),
             "GET /temp?temperature=%.2f&moisture=%.2f&gas_level=%.2f&sensor_id=%d HTTP/1.1\r\nHost: 192.168.15.163\r\n\r\n",
             sensors_reading.temperature,
             sensors_reading.moisture,
             sensors_reading.gas_level,
             SENSOR_ID);

    // Envia a requisição
    if (tcp_write(pcb, request, strlen(request), TCP_WRITE_FLAG_COPY) != ERR_OK)
    {
        display_text("Erro ao enviar dados");
        tcp_abort(pcb); // Aborta em caso de erro
        tcp_close(pcb); // Fecha
        return;         // Sai
    }

    // Sinaliza que os dados foram enviados (para o lwIP)
    if (tcp_output(pcb) != ERR_OK)
    {
        display_text("Erro (tcp_output)");
        tcp_abort(pcb); // Aborta em caso de erro
        tcp_close(pcb); // Fecha
        return;         // Sai
    }

    tcp_recv(pcb, tcp_client_recv);
}

/**
 @brief Desenha a parte inferior (estática) da tela do OLED.
 */
void draw_screen_botton()
{
    int text_len = strlen(bottom_screen_text);
    ssd1306_draw_string(&display, (SCREEN_WIDTH - text_len * 6) - 2, SCREEN_BOTTOM_LIMIT - 16 / 2, 1, bottom_screen_text);
    ssd1306_draw_square(&display, 0, SCREEN_BOTTOM_LIMIT, SCREEN_WIDTH, SCREEN_HEIGHT - SCREEN_BOTTOM_LIMIT);
    ssd1306_show(&display);
}

/**
 @brief Desenha todos os componentes estáticos da tela.
 */
void draw_screen_components()
{
    draw_screen_botton();
}

/**
 * @brief Tenta conectar-se à rede Wi-Fi.
 *
 * Inicializa o hardware Wi-Fi, habilita o modo STA e tenta conectar-se à rede
 * Wi-Fi especificada usando as credenciais fornecidas.
 *
 * @return Retorna 0 se a conexão for bem-sucedida, -1 se falhar.
 */
int connect_to_wifi()
{
    if (cyw43_arch_init())
    {
        display_text("Erro ao iniciar wifi!\n");
        return -1;
    }

    cyw43_arch_enable_sta_mode();

    int retries = 5;

    while (retries > 0)
    {
        display_text("Conectando...\n");
        int result = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, WIFI_CONNECT_TIMEOUT);

        if (!result)
        {
            char sucess_text[40];
            sprintf(sucess_text, "Conectado à %s", WIFI_SSID);
            display_text(sucess_text);
            return 0;
        }

        char failure_text[40];
        sprintf(failure_text, "Erro: %d", result);
        display_text(failure_text);
        sleep_ms(500);
        retries -= 1;
    }
    return -1;
}

int main()
{
    // Inicializa todos os subsistemas (stdio, ADC, I2C, etc.)
    init_all();

    // Inicializa sensor DHT11
    dht_init(&dht, DHT_MODEL, pio0, DHT_PIN, true /* habilita pull_up interno*/);

    // Funções do display
    clear_display();

    // Inicializando e conectando ao módulo wi-fi
    if (!connect_to_wifi())
    {
        display_text("Conexão mal-sucedida...");
    }

    while (true)
    {
        // Criando estrutura PCB para conexão TCP
        struct tcp_pcb *pcb = create_tcp_struct();

        read_sensors(); // Lê os sensores
        display_sensor_data();
        send_data_to_server(pcb); // Envia os dados para o servidor
        sleep_ms(5000);           // Aguarda cinco segundos
        clear_display();
    }

    return 0;
}

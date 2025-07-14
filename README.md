# Comunicação do projeto de monitoramento do Rio Una
Esse repositório contém a parte referente a comunicação do projeto de monitoramento do Rio Una.

## 0. Visão Geral
O projeto implementa uma comunicação SPI entre uma placa STM (atuando como Master) e uma ESP32 (atuando como Slave). A STM envia dados de sensores (que serão utilizados no monitoramento do Rio Una) via SPI para a ESP32, que por sua vez processa e encaminha essas informações para uma API HTTP utilizando o método POST.

## 1. Placas e configurações utilizadas

Foram utilizadas uma placa STM Nucleo H753ZI e uma ESP32 DEVKIT V1 de 38 pinos.

* A pasta SPI_do_zero contém tanto o código quanto a configuração do projeto para a STM.
* O arquivo SKECTHCFUNCIDOSNASDO.ino contém tanto o código quanto a configuração do projeto para a ESP32.

As coneções entre as placas estão descritas na tabela abaixo:

| ESP32 | STM (Nucleo H753ZI) | Função      |
| ----- | --------------------- | ----------- |
| G5    | D12                   | MOSI        |
| G18   | D13                   | CS          |
| G23   | D11                   | MOSI        |
| GND   | GND                   | Terra (GND) |

## 2. Inicializando o projeto

#### Pré-requisitos para inicialização:
* Instalação da STM32CubeIDE e Arduino IDE. 
* Placas conectadas.
* Servidor pronto para receber as requisições.

#### Passo a passo para garantir funcionamento correto do projeto:
Passo 0: Ao configurar o servidor da REST API, altere o código da ESP32 para refletir o endereço correto da API.
Passo 1: Rode o código da ESP32, para que ela fique em estado de espera de recebmento de dados.
Passo 2: Siga o passo a passo do Serial Monitor da ESP32 para conecta-la em uma rede wifi com acesso a internet.
Passo 3: Rode o código da STM.                                                                                                                                                                                              
Passo 4: Observe se os dados estão chegando corretamente na ESP32.                                                                                                                                            
Passo 5: Observe se os dados estão chegando corretamente no servidor.

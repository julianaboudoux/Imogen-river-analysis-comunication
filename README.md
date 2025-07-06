# Comunicação do projeto de monitoramento do Rio Una
Esse repositório contém a parte referente a comunicação do projeto de monitoramento do Rio Una.

O projeto implementa uma comunicação SPI entre uma placa STM (atuando como Master) e uma ESP32 (atuando como Slave). A STM envia dados de sensores (que serão utilizados no monitoramento do Rio Una) via SPI para a ESP32, que por sua vez processa e encaminha essas informações para uma API HTTP utilizando o método POST.


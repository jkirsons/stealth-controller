# Stealth Controller
A Field Oriented Control Brushless DC Motor Controller.

With 8A driver, Magnetic Sensor, Microcontroller, Two-Wire Automotive Interface

![Board](/Resources/Driver%200.2.png)

---
BOM

Ref | Qnty | Value | Size | LCSC #
--- | --- | --- | --- | --- 
C3, C21, C22, C23, C24, C25, C26, |7|22uF 25v|0805|[C45783](https://lcsc.com/product-detail/Multilayer-Ceramic-Capacitors-MLCC-SMD-SMT_Samsung-Electro-Mechanics-CL21A226MAQNNNE_C45783.html)
C1, C15, C16, |3|22uF 10v|0603|[C86295](https://lcsc.com/product-detail/Multilayer-Ceramic-Capacitors-MLCC-SMD-SMT_Samsung-Electro-Mechanics-CL10A226MP8NUNE_C86295.html)
C9, C18, C20, |3|10uF 10v|0603|[C19702](https://lcsc.com/product-detail/Multilayer-Ceramic-Capacitors-MLCC-SMD-SMT_Samsung-Electro-Mechanics-CL10A106KP8NNNC_C19702.html)
C11, |1|2.2uF 50v|0805|[C337444](https://lcsc.com/product-detail/Others_Taiyo-Yuden-UMK212ABJ225KG-T_C337444.html)
C4, C6, C10, C13, |4|1uF 16v|0603|[C318643](https://lcsc.com/product-detail/Multilayer-Ceramic-Capacitors-MLCC-SMD-SMT_Samsung-Electro-Mechanics-CL10A105KO5LNNC_C318643.html)
C2, C7, C8, C12, C14, C17, C19, |7|100nF 50v|0603|[C490602](https://lcsc.com/product-detail/Multilayer-Ceramic-Capacitors-MLCC-SMD-SMT_KEMET-C0603C104J5RAC_C490602.html)
C5, |1|10nF 50v|0603|[C128615](https://lcsc.com/product-detail/Multilayer-Ceramic-Capacitors-MLCC-SMD-SMT_Walsin-Tech-Corp-0603B103J500CT_C128615.html)
L1, |1|15uH|1210|[C478845](https://lcsc.com/product-detail/Inductors-SMD_Taiyo-Yuden-BRL3225T150K_C478845.html)
R1, |1|22|0603|[C269708](https://lcsc.com/product-detail/Chip-Resistor-Surface-Mount_TyoHM-RMC0603221-N_C269708.html)
R2|1|5.1k|0603|[C103696](https://lcsc.com/product-detail/Chip-Resistor-Surface-Mount_RALEC-RTT03512JTP_C103696.html)
R3, R4, |2|10K|0603|[C328335](https://lcsc.com/product-detail/Chip-Resistor-Surface-Mount_Resistor-Today-AECR0603F10K0K9_C328335.html)
R5, |1|120|0603|[C22787](https://lcsc.com/product-detail/Chip-Resistor-Surface-Mount_UNI-ROYAL-Uniroyal-Elec-0603WAF1200T5E_C22787.html)
IC1, |1|ESP32-PICO-V3|ESP32-PICO-V3|[C967022](https://lcsc.com/product-detail/RF-Transceiver-ICs_Espressif-Systems-ESP32-PICO-V3_C967022.html)
IC2, |1|LMR36506|LMR36506|[TI](https://www.ti.com/store/ti/en/p/product/?p=LMR36506R3RPER)
IC3, |1|MA702GQ-P|MA702GQ-P|[Mouser](https://www.mouser.ch/ProductDetail/Monolithic-Power-Systems-MPS/MA702GQ-P?qs=%2Fha2pyFadugcckvyIxaK%2FPg3fxrEo4S43%2FImNSAiLmAbp7VlBKN7pA%3D%3D)
IC4, |1|SN65HVD230QDRG4|SN65HVD230QDRG4|[C16468](https://lcsc.com/product-detail/CAN-ICs_Texas-Instruments-SN65HVD230QDR_C16468.html)
U1, |1|DRV8316R|DRV8316R|[TI](https://www.ti.com/product/DRV8316?keyMatch=DRV8316&tisearch=search-everything&usecase=GPN#order-quality)
Reset1, Boot1, |2|SW_Push_Dual|XKB Connectivity TS-1177-B-B-B|[C561510](https://lcsc.com/product-detail/Tactile-Switches_XKB-Connectivity-TS-1177-B-B-B_C561510.html)
UART1, TWAI1, TWAI2, |3|UART/TWAI|S3B-ZR-SM4A-TF|[C158001](https://lcsc.com/product-detail/Wire-To-Board-Wire-To-Wire-Connector_JST-Sales-America-B3B-ZR-LF-SN_C158001.html)
J1, |1|XT30PB|XT30PB|[C428721](https://lcsc.com/product-detail/Power-Connectors_Changzhou-Amass-Elec-XT30UPB-M_C428721.html)

---

Links:
[Simple FOC](https://simplefoc.com/)

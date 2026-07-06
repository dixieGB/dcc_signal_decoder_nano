# dcc_signal_decoder
Self-learning DCC accessory decoder for UK colour-light signals (2–4 aspect + route indicator). Learns its own address from the command station — no PC needed after setup. Arduino/ATmega328P firmware with EEPROM config, PWM brightness control, and an inter-board "Aspect Link" for automatic block signalling.

## Documentation
- [Firmware Configuration & Maintenance Guide](DCC_Signal_Decoder_Firmware_Configuration_Guide_v1_4.docx) — design rationale, pin assignments, and build-time configuration options.
- [User Guide](DCC_Signal_Decoder_User_Guide_v1_4.docx) — day-to-day operation once a decoder is installed and learned.
- [Project Status](DCC_Signal_Decoder_Project_Status.md) — current state, known limitations, and what's still to be tested.

## Firmware
The main sketch is [`DCC_Signal_Decoder_v1_4.ino`](DCC_Signal_Decoder_v1_4.ino).

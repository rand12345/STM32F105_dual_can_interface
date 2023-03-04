## Development of an inexpensive dual can bus gateway 

* [X] Async (some blocking) dual can bus
* [X] Async USART
* [X] Single PWM output
* [X] Single GPIO output
* [X] JSON MQTT data output on UART for nodeMCU

Hardware details can be found here: https://github.com/EliasKotlyar/Canfilter

Currently parsing an EV battery library - WIP.

### Todo:

* [ ] Interupt driven can bus or async
* [ ] [Multiprio](https://github.com/embassy-rs/embassy/blob/master/examples/stm32f4/src/bin/multiprio.rs)

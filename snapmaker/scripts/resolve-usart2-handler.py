"""Keep the STM32 framework's USART2 ISR from shadowing Snapmaker's ISR.

The Snapmaker controller uses a custom USART2 interrupt handler in
MarlinSerial.cpp for its DMA / IDLE-line receive path. Arduino_Core_STM32 1.9.0
also provides an unconditional USART2_IRQHandler in uart.c, so both strong
symbols otherwise reach the linker.

Rename only the framework-owned handler while compiling uart.c. The vector
table can then resolve USART2_IRQHandler to the Snapmaker implementation. This
is deliberately scoped to one framework translation unit and does not modify
the installed PlatformIO package.
"""

Import("env")


def use_snapmaker_usart2_handler(build_env, node):
    source_path = node.srcnode().get_path().replace("\\", "/")
    if not source_path.endswith("/libraries/SrcWrapper/src/stm32/uart.c"):
        return node

    uart_env = build_env.Clone()
    uart_env.Append(
        CPPDEFINES=[("USART2_IRQHandler", "STM32_USART2_IRQHandler")]
    )
    return uart_env.Object(node)


env.AddBuildMiddleware(use_snapmaker_usart2_handler)

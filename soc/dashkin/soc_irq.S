#include <zephyr/toolchain.h>

GTEXT(__soc_handle_irq)

/*
 * SOC-specific function to handle pending IRQ number generating the interrupt.
 * Exception number is given as parameter via register a0.
 */
SECTION_FUNC(exception.other, __soc_handle_irq)
	/*
	 * The the MIP CSR on the DASHKIN is read-only and can thus not be used for
	 * clearing a pending IRQ. Instead we disable the IRQ in the MIE CSR and
	 * re-enable it (if it was enabled when clearing).
	 */
	li t1, 1
	sll t0, t1, a0
	csrrc t2, mie, t0
	and t1, t2, t0
	csrrs t2, mie, t1

	/* Return */
	ret

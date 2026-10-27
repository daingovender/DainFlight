# Software Development Plan (SDP) & Verification Plan (SVP)
## System: DainFlight Primary Flight Control System (FCS)
## Standard: RTCA DO-178C Level A / MISRA C:2012

---

### Part 1: Software Development Plan (SDP)

#### 1. Coding Standard: MISRA C:2012 Mandatory Rules
DainFlight strictly adheres to the following core avionics programming rules:
* **Rule 4.6:** Typedefs indicating size and signedness must be used in place of basic numerical types (`float32_t`, `boolean_t`, `uint32_t`, `int32_t`).
* **Rule 8.9:** An object should be defined at block scope if its identifier only appears within that block.
* **Rule 11.4 / 11.5:** No conversion between pointer to object and integer or void pointer.
* **Rule 13.3:** Side effects shall not be used in expressions (e.g. no `x = y++`).
* **Rule 14.3:** Controlling expressions shall not be invariant.
* **Rule 15.5:** A function shall have a single point of exit at the end of the function.
* **Rule 17.2:** Functions shall not call themselves, either directly or indirectly (zero recursion).
* **Rule 21.3:** The memory allocation and deallocation functions of `<stdlib.h>` shall not be used (no `malloc`, `calloc`, `free`, `realloc`).

#### 2. Defensive Programming Strategy
* Every pointer parameter is validated for non-NULL before dereference.
* Every floating-point parameter is validated for finite range before arithmetic.
* Every division operation is preceded by a non-zero denominator assertion/check.
* Trigonometric arguments are bounded to mathematical definition domains.

---

### Part 2: Software Verification Plan (SVP)

#### 1. Structural Coverage Objectives (DO-178C Table A-7 Level A)
To achieve Level A certification, verification must demonstrate:
* **100% Statement Coverage:** Every source statement executed in test.
* **100% Decision Coverage:** Every decision evaluated to TRUE and FALSE.
* **100% Modified Condition/Decision Coverage (MC/DC):** Every boolean condition independently demonstrated to affect decision outcome.

#### 2. Verification Methods
* **Requirements-Based Testing:** Every Low-Level Requirement (LLR) has dedicated unit test cases.
* **Boundary Value Analysis (BVA):** Testing inputs at minimum, nominal, maximum, and out-of-bounds extremes.
* **Robustness & Error Injection Testing:** Testing response to NULL pointers, NaNs, infinities, radio drops, and sensor spikes.

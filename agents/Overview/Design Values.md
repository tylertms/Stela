---
tags: [stela, overview, conventions]
---
# Design Values

Four habits explain why the code looks the way it does. Knowing them makes the code much easier to read.

## 1. Stop instead of guessing
When Stela meets something it does not understand, it **stops and explains**, rather than making up an answer.

Example: macOS reads a hardware setting Stela has not implemented. A lazy emulator would return 0 and hope. Stela returns an error that says exactly which setting was read.

Why: a wrong guess causes a strange crash much later, far from the real cause. A clear stop points right at the problem. The code calls this a *refusal*. See [[Refusals]].

## 2. Never change macOS
Stela does not edit Apple's files. If macOS expects something, Stela provides it. Comments often quote the exact macOS version and location where a behavior was observed.

## 3. One correct version, plus faster copies
For tricky behavior there is usually:
- one **slow but clearly correct** implementation, which is the definition, and
- one or more **fast** implementations that must give identical results.

If the fast one cannot handle a case, it hands over to the slow one. You will see this with floating-point math and special Apple instructions. See [[Translator Fast Paths]].

## 4. Say where facts come from
Hardware behavior is not obvious, so comments name their source: the Arm manual, the Intel manual, Apple's open-source kernel, or what Linux does. They also say when something is a guess.

> [!tip] When you write code here
> - Don't return a default for an unknown case. Refuse.
> - Write down where you learned a hardware fact.
> - Keep the slow, correct version as the source of truth.
>
> More in [[Conventions]].

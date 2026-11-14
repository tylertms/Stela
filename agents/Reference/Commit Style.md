---
tags: [stela, conventions, git]
---
# Commit Style

A *commit message* describes a change in git. This project has a clear style. Look at `git log --oneline` to see it.

## The format
```
type(scope): one full sentence describing the change
```

- **type** — what kind of change
- **scope** — which part of the project
- **the sentence** — what you did, specifically

## Types
| Type | Use it for |
|---|---|
| `feat` | Something new |
| `fix` | A bug fix |
| `perf` | Making something faster |
| `docs` | Comments or documentation only |
| `style` | Formatting only |
| `chore` | Housekeeping |

## Scopes
The crate or area you touched: `engine`, `compiler`, `boot`, `soc`, `storage`, and so on.

## Real examples from this project
```
fix(boot): give the host disk completion signal vector its interrupt gate, and check at compile time that every named vector has one

perf(compiler): decode WKdm pages with no branch on each word's tag, selecting its candidates by masks

feat(engine): count ITLB entries TLBIs drop in every context by why: the upper half, no ASID, or a global page
```

## What to notice
- **They are long and specific.** Not "fix bug" but exactly what was fixed and how.
- **One change per commit.** Don't mix a bug fix with a new feature.
- **Measure first, then speed up.** Many `feat` commits add a counter or timer. A `perf` commit usually follows, justified by those numbers.

## Good and bad
| Avoid | Prefer |
|---|---|
| `fix stuff` | `fix(soc): refuse a GPIO write wider than the register instead of truncating it` |
| `faster` | `perf(engine): look up a block by page before hashing its bytes` |

> [!tip] If you can't describe it in one sentence
> The commit is probably doing too much. Split it.

Related: [[Conventions]], [[Build and Check]].

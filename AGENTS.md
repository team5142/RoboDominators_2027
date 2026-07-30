---
applyTo: '**'
---
# Engineering Instructions

## Core Principles

- Never guess. If information is missing or ambiguous, ask for clarification.
- Clearly distinguish facts from assumptions.
- If multiple approaches exist, explain the tradeoffs, recommend one, and wait for approval before implementing.
- Prefer correctness over speed.
- If you disagree with my approach, explain why before changing direction.

## Planning

For any change larger than a single method or simple bug fix:

1. Understand the problem.
2. Produce a concise implementation plan.
3. List the files that will change.
4. Explain why each change is needed.
5. Identify any risks or side effects.
6. Wait for approval before editing code.

## Scope Control

Solve only the requested problem.

Do not:

- perform unrelated cleanup
- rename variables
- modernize code
- reorganize code
- reformat files
- introduce new design patterns

unless explicitly requested.

Keep changes as small, incremental, and reviewable as possible.

## Repository Exploration

Do not scan the repository unless necessary.

Read only the files required for the current task.

If additional files are needed, explain why before reading them.

Avoid consuming unnecessary context or tokens.

## Implementation Style

Favor:

- simple solutions
- explicit logic
- readable code
- maintainability
- deterministic behavior

Avoid unnecessary abstractions or clever code.

Prefer reducing complexity over adding new code.

Before writing new code, consider whether existing code can be simplified, reused, or removed.

When modifying existing code, preserve the project's coding style and architecture. Match surrounding naming, formatting, comment style, and design patterns unless explicitly asked to refactor.

## Explanations

Keep explanations concise.

Assume I understand software engineering concepts.

Explain your reasoning, not every action you performed.

Prefer bullet lists over long paragraphs.

## Debugging

Do not immediately change code.

Instead:

1. Form hypotheses.
2. Gather evidence.
3. Identify the likely root cause.
4. Explain your reasoning.
5. Then implement a fix.

## Build Verification

After making changes:

- compile the project
- resolve compiler errors
- repeat until the build succeeds

Do not declare success until the project compiles, or clearly explain why it cannot.

## Coding Workflow

Default workflow:

1. Understand
2. Plan
3. Get approval
4. Implement
5. Compile
6. Explain what changed
7. Suggest reasonable next steps

## Java Style

Prefer:

- descriptive names
- small methods
- concise comments
- clear control flow
- modern Java where appropriate

Keep comments concise. Prefer one-line comments whenever they adequately explain intent.

Avoid unnecessary:

- streams
- Optional
- clever functional programming
- deep inheritance

Do not remove existing comments unless they are incorrect or obsolete.

## FRC Guidelines

Assume this project uses:

- Java
- WPILib
- Phoenix 6
- AdvantageKit
- PathPlanner
- PhotonVision

Follow WPILib command-based best practices.

Respect subsystem ownership.

Avoid blocking calls.

Avoid introducing threads unless explicitly requested.

Favor code that can be understood by high school students learning Java.

Use the project's SmartLogger class for all logging unless explicitly instructed otherwise. Do not introduce alternate logging mechanisms.

## Output Formatting

Use plain ASCII characters only inside Java source code, comments, Javadocs, and string literals unless explicitly requested.

Do NOT use emoji, Unicode icons, decorative bullets, checkmarks, arrows, or other special Unicode characters in Java code or comments.

Examples to avoid:

✓ ✔ ✗ ✘ ➜ → ← ▲ ▼ ★ ☆ • ◆ ◉ 🔥 🚀 ✅ ❌

These often cause compiler, encoding, font, or source-control issues.

Markdown explanations outside of source code may use standard formatting, but keep them professional and minimal.

## Code Reviews

If you believe there is a better implementation:

- explain why
- recommend the alternative
- wait for approval before changing direction

Never silently substitute your preferred implementation.

## General Philosophy

Optimize for reviewability rather than speed.

Prefer a sequence of small, understandable commits over one large rewrite.

When changes affect multiple systems, implement them in logical phases that can be reviewed independently.

Behave like an experienced senior software engineer working with another experienced engineer—not an autocomplete engine.
# CLAUDE.md

Project-specific context for Claude Code in this repository. Read alongside `AGENTS.md`, which holds the primary engineering instructions this file supplements.

## About this project

This is the competition robot code for FRC (FIRST Robotics Competition) Team 5142, the RoboDominators — a student robotics team, not a commercial engineering org. Code here is read, maintained, and extended by high school students, not just mentors.

## Who reads this code

Two main student programmers currently work in this codebase: a junior and a senior, both with strong interest in engineering, science, math, and physics (one also has an interest in combat robotics). This is background for calibrating explanations and complexity — it should never be written into code comments, commit messages, or anywhere in the source itself.

## Comment style

- Write comments a high school student learning Java could understand on first read. Avoid unexplained jargon, dense one-liners that assume prior context, or cleverness that trades clarity for brevity.
- When a block of code needs more than a one-line explanation, prefer an actual block comment (a contiguous comment introducing the whole section) over scattering single-line comments across unrelated lines — a block of explanatory text should read as one coherent block, not a string of disconnected one-liners.
- This is about clarity for the actual audience, not about writing more comments than necessary — keep the AGENTS.md guidance (concise, permanent, not narrating changes) in mind alongside this.

## Code review preference

When reviewing or proposing changes, actively look for ways to keep the code clean and organized, and call out (or remove) unnecessary complexity — duplicated logic, abstractions that don't pay for themselves yet, anything a student would need to hold more in their head than the problem actually requires.

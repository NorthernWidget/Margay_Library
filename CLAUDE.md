# Margay_Library

Library for the Margay data logger (I²C controller).

## Standards

Follow the NW library standards for all work here. Two sources:
- **Within the NorthernWidget workspace:** the root `CLAUDE.md` (one level above `github/`) has the full checklist and code conventions.
- **Fresh clone / other location:** fetch [NorthernWidget/.github/RELEASING.md](https://github.com/NorthernWidget/.github/blob/main/RELEASING.md) for the release checklist and [NorthernWidget/.github/CONTRIBUTING.md](https://github.com/NorthernWidget/.github/blob/main/CONTRIBUTING.md) for naming and versioning conventions.

Margay is a **controller** (I²C controller/master), not a sensor. It calls `Wire.begin()` with no address. It does not implement the Schema 1 sensor register map; its Schema 1 entry in [NW-Device-Specification](https://github.com/NorthernWidget/NW-Device-Specification) is identity-only.

## Hard rule

**Never** create a git tag, GitHub release, push to a shared remote, or submit to any external registry (Zenodo, Arduino Library Manager, etc.) unless explicitly asked in the current message. If in doubt, ask.

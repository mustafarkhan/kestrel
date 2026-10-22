# 0001. The language model plans; it never pilots

**Context.** Language control of an aircraft is attractive and risky: models
are slow, non-deterministic and confidently wrong.

**Decision.**
- The model's only output is a `Mission` JSON document. It is constrained with
  Ollama structured outputs (the pydantic JSON Schema is passed as `format`), then
  re-validated with pydantic. Constraint reduces bad output; validation is what
  is relied on.
- Invalid output gets one repair round with the validation error; then the
  deterministic rule parser tries; if both fail, the command is rejected. Nothing
  is guessed.
- Site names the model invents are rejected (it must use the configured list).
- `stop`, `abort`, `land now` are matched by regex and never reach the model.
- Local model by default (`qwen2.5:3b`): no network dependency in the air and no
  operational data leaving the machine.

**Consequences.** The rest of the stack never sees model output, only validated
plans, and works identically with or without a model available.

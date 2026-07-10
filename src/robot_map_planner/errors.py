from __future__ import annotations


class PlannerError(RuntimeError):
    def __init__(self, code: str, message: str, *, status_code: int = 400) -> None:
        super().__init__(message)
        self.code = code
        self.status_code = status_code


def translate_core_error(exc: Exception) -> PlannerError:
    text = str(exc)
    if ":" in text:
        code, message = text.split(":", 1)
        if code in {"INVALID_PCD", "INVALID_CONFIG"}:
            return PlannerError(code, message.strip())
    return PlannerError("INTERNAL_ERROR", text, status_code=500)

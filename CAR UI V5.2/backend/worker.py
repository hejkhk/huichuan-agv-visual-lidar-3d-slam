from __future__ import annotations

import traceback
from typing import Any, Callable

from PySide6.QtCore import QObject, QRunnable, Signal, Slot


class WorkerSignals(QObject):
    finished = Signal(object)
    error = Signal(str)


class ApiWorker(QRunnable):
    def __init__(self, function: Callable[..., Any], *args: Any):
        super().__init__()
        self.function = function
        self.args = args
        self.signals = WorkerSignals()

    @Slot()
    def run(self) -> None:
        try:
            self.signals.finished.emit(self.function(*self.args))
        except Exception:
            self.signals.error.emit(traceback.format_exc())

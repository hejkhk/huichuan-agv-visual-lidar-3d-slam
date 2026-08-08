import subprocess

from backend.system_audio import SystemAudio


def _completed(
    arguments: list[str],
    *,
    stdout: str = "",
    stderr: str = "",
    returncode: int = 0,
) -> subprocess.CompletedProcess[str]:
    return subprocess.CompletedProcess(
        arguments,
        returncode,
        stdout=stdout,
        stderr=stderr,
    )


def test_pipewire_volume_is_read_as_percent(monkeypatch):
    audio = SystemAudio()
    monkeypatch.setattr(
        "backend.system_audio.shutil.which",
        lambda command: f"/usr/bin/{command}",
    )
    monkeypatch.setattr(
        audio,
        "_run",
        lambda arguments: _completed(arguments, stdout="Volume: 0.37\n"),
    )

    assert audio.read_volume() == (37, "")


def test_pipewire_volume_set_unmutes_default_sink(monkeypatch):
    audio = SystemAudio()
    calls = []
    monkeypatch.setattr(
        "backend.system_audio.shutil.which",
        lambda command: f"/usr/bin/{command}",
    )

    def run(arguments):
        calls.append(arguments)
        return _completed(arguments)

    monkeypatch.setattr(audio, "_run", run)
    assert audio.set_volume(64) == (True, "")
    assert calls == [
        [
            "wpctl",
            "set-volume",
            "@DEFAULT_AUDIO_SINK@",
            "64%",
        ],
        [
            "wpctl",
            "set-mute",
            "@DEFAULT_AUDIO_SINK@",
            "0",
        ],
    ]


def test_alsa_is_used_when_wpctl_is_unavailable(monkeypatch):
    audio = SystemAudio()
    monkeypatch.setattr(
        "backend.system_audio.shutil.which",
        lambda command: "/usr/bin/amixer" if command == "amixer" else None,
    )
    monkeypatch.setattr(
        audio,
        "_run",
        lambda arguments: _completed(
            arguments,
            stdout="Front Left: Playback 32768 [50%] [on]\n",
        ),
    )

    assert audio.read_volume() == (50, "")

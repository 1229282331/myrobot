"""Media catalogue for finished project sessions; never imports a device driver."""
from pathlib import Path
import re
import threading
import time

from .camera_archive import atomic_json, read_atomic_json
from .episodes import checked_id


class RetainedEpisodes:
    def __init__(self, sessions, *, recovery_id=None, episode=None):
        self.root = Path(sessions).resolve(strict=True)
        self.recovery_id = recovery_id
        self.episode = checked_id(episode) if episode is not None else None
        self.receipt_lock = threading.Lock()

    def state(self):
        return dict(phase='idle', episode=None, recovery_only=True, recovery_id=self.recovery_id)

    def _read(self, path):
        data = read_atomic_json(self._inside(path))
        if not isinstance(data, dict): raise ValueError('Retained metadata must be an object')
        return data

    def _inside(self, path):
        resolved = path.resolve(strict=True)
        # Reject symlinks/junctions even when they point elsewhere inside the
        # catalogue: one retained file must have one unambiguous identity.
        if resolved != path or not resolved.is_relative_to(self.root):
            raise ValueError('Retained path is not a direct project file')
        return resolved

    def catalogue(self):
        found = {}; errors = []
        for session in sorted(self.root.iterdir()):
            if not re.fullmatch(r'\d{8}-\d{6}-[a-f0-9]{8}', session.name):
                continue
            try:
                self._inside(session)
                owner = self._read(session/'owner.json')
                result = self._read(session/'result.json')
                if owner.get('session') != session.name or result.get('session') != session.name:
                    raise ValueError('Session ownership mismatch')
                if type(result.get('finished_ns')) is not int or result['finished_ns'] <= 0:
                    raise ValueError('Session has no finished receipt')
                episodes = self._inside(session/'episodes')
            except FileNotFoundError:
                continue  # Running/unrecorded sessions are outside recovery.
            except (OSError, ValueError, TypeError) as exc:
                errors.append(dict(session=session.name, error=str(exc))); continue
            for directory in sorted(episodes.iterdir()):
                try:
                    checked_id(directory.name)
                    self._inside(directory)
                    if not directory.is_dir(): continue
                    # Include unsealed and damaged identities in collision
                    # detection; never pick a convenient duplicate silently.
                    found.setdefault(directory.name, []).append(directory)
                except (OSError, ValueError) as exc:
                    errors.append(dict(session=session.name, error=str(exc)))
        items = []; paths = {}
        for episode, directories in sorted(found.items(), reverse=True):
            if self.episode is not None and episode != self.episode: continue
            if len(directories) != 1:
                errors.append(dict(episode=episode, error='Duplicate retained episode identifier')); continue
            directory = directories[0]
            try:
                item = self._read(directory/'transfer.json')
                if item.get('episode') != episode: raise ValueError('Episode identity mismatch')
                digest = item.get('bundle_sha256')
                if not digest: continue  # Not sealed; never acknowledge partial media.
                if not isinstance(digest, str) or not re.fullmatch('[a-f0-9]{64}', digest):
                    raise ValueError('Invalid sealed bundle digest')
                if item.get('phase') not in ('pending', 'verified'):
                    raise ValueError('Invalid sealed transfer phase')
                self._inside(directory/'bundle.zip')
                summary = self._read(directory/'summary.json')
                if summary.get('episode') != episode: raise ValueError('Summary identity mismatch')
                items.append(dict(item, summary=summary, retained_session=directory.parent.parent.name))
                paths[episode] = directory
            except (OSError, ValueError, TypeError) as exc:
                errors.append(dict(episode=episode, error=str(exc)))
        return dict(current=self.state(), episodes=items, catalogue_errors=errors), paths

    def listing(self):
        return self.catalogue()[0]

    def directory(self, episode):
        checked_id(episode)
        _, paths = self.catalogue()
        if episode not in paths: raise ValueError('Sealed retained episode unavailable or ambiguous')
        return paths[episode]

    def transferred(self, episode, digest):
        with self.receipt_lock:
            directory = self.directory(episode)
            saved = read_atomic_json(directory/'transfer.json')
            if not digest or digest != saved['bundle_sha256']:
                raise ValueError('回传校验标识不一致')
            # Only the transfer receipt changes. Outcome, source media, raw
            # feedback and the immutable bundle remain byte-for-byte intact.
            atomic_json(directory/'transfer.json', dict(saved, phase='verified', x5_ack_ns=time.perf_counter_ns()))
            return dict(self.state(), verified_episode=episode)

    def close(self):
        pass

"""
Debug aid only -- auto-imported by the interpreter because launch.sh puts
this directory on PYTHONPATH when LAUNCH_DEBUG=true. Wraps
Node._perform_substitutions so a parameter-evaluation failure (e.g. "got
'()' of type 'tuple'") prints which package/executable/name Node it came
from -- the stock launch traceback never says.
"""
try:
    from launch_ros.actions import node as _node_mod

    _orig = _node_mod.Node._perform_substitutions

    def _patched(self, context):
        try:
            return _orig(self, context)
        except Exception as exc:
            print(
                f"\n>>> DEBUG: parameter evaluation failed on Node "
                f"package={getattr(self, '_Node__package', None)!r} "
                f"executable={getattr(self, '_Node__node_executable', None)!r} "
                f"name={getattr(self, '_Node__node_name', None)!r} "
                f"namespace={getattr(self, '_Node__node_namespace', None)!r}: {exc}\n",
                flush=True,
            )
            raise

    _node_mod.Node._perform_substitutions = _patched
except Exception as e:  # pragma: no cover -- debug aid only, never block a real launch
    print(f"sitecustomize debug patch failed to install: {e}", flush=True)

"""The complete UI patch comparison must never accept unreviewed local edits."""
from pathlib import Path
import importlib.util
import subprocess
import tempfile
import unittest

spec=importlib.util.spec_from_file_location('prepare_ui',Path(__file__).resolve().parents[1]/'scripts/prepare_upstream.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class CompleteChainTests(unittest.TestCase):
    def test_overlapping_patches_and_local_change_preservation(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            def git(*args):
                return subprocess.run(['git',*args],cwd=root,check=True,capture_output=True)
            git('init','-q')
            source=root/'page.txt';source.write_text('upstream\n',encoding='utf8',newline='\n')
            git('add','page.txt');git('-c','user.name=UI fixture','-c','user.email=ui@example.invalid','commit','-qm','fixture')
            first=root/'first.patch';second=root/'second.patch'
            first.write_text('--- a/page.txt\n+++ b/page.txt\n@@ -1 +1 @@\n-upstream\n+intermediate\n',encoding='utf8',newline='\n')
            second.write_text('--- a/page.txt\n+++ b/page.txt\n@@ -1 +1 @@\n-intermediate\n+final\n',encoding='utf8',newline='\n')
            source.write_text('final\n',encoding='utf8',newline='\n')
            self.assertTrue(module.matches_complete_patch_chain(root,[first,second]))
            source.write_text('user edit\n',encoding='utf8',newline='\n')
            self.assertFalse(module.matches_complete_patch_chain(root,[first,second]))
            self.assertEqual(source.read_text(encoding='utf8'),'user edit\n')
            # The user's actual index is untouched by the temporary comparison.
            self.assertEqual(git('show',':page.txt').stdout,b'upstream\n')

if __name__=='__main__':unittest.main()

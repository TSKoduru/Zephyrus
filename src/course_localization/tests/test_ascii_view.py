"""Read-only terminal rendering and initial walk-launch contracts."""
import ast
import io
from pathlib import Path
import unittest
import numpy as np
from course_localization.ascii_view import AsciiMap, Terminal, LiveLogView


class AsciiTest(unittest.TestCase):
    def test_center_particles_walls_and_outside(self):
        canvas=AsciiMap(21,11,(-1,1,-1,1))
        canvas.add_wall([-.8,-.8],[-.8,.8])
        canvas.add_wall([-.5,-.5],[.5,.5])
        lines,outside=canvas.lines([[0,0],[.5,0],[100,0]], [0,0,0,0])
        text='\n'.join(lines)
        self.assertEqual(outside,1)
        self.assertEqual(text.count('#'),1)
        self.assertEqual(text.count('.'),1)
        self.assertIn('/',text)
        row,col=canvas.cell(0,0)
        self.assertEqual(lines[row+1][col+1],'#')

    def test_same_terminal_panel_live_stale(self):
        view=LiveLogView(Path(__file__).resolve().parents[1]/'map')
        self.assertIn('STALE / WAITING',view.text(10))
        view.update([0,0,.8,0],np.array([[0,0,.8,0],[.5,.5,.8,0]]),10.)
        self.assertIn('WALK LOCALIZATION: LIVE',view.text(10.1))
        self.assertIn('STALE / WAITING',view.text(12.))
        self.assertIn('STALE / WAITING',view.text(10.1,'lost tracking'))
        self.assertLess(len(view.text(10.1).encode()),2200)
        self.assertNotIn('\033',view.text(10.1))

    def test_optional_terminal_sends_only_changes(self):
        output=io.StringIO();terminal=Terminal(output)
        terminal.draw(['abc','def']);before=len(output.getvalue())
        terminal.draw(['abc','def']);self.assertEqual(len(output.getvalue()),before)
        terminal.draw(['abc','ghi']);self.assertNotIn('abc',output.getvalue()[before:])
        terminal.close();self.assertIn('\033[?25h',output.getvalue())

    def test_walk_launch_only_localization(self):
        p=Path(__file__).resolve().parents[1]/'launch/walk_test.launch.py'
        tree=ast.parse(p.read_text());executables=[]
        for call in ast.walk(tree):
            if isinstance(call,ast.Call) and isinstance(call.func,ast.Name) and call.func.id=='Node':
                executables.extend(ast.literal_eval(k.value) for k in call.keywords if k.arg=='executable')
        self.assertEqual(executables,['localize'])
        self.assertIn("'ascii_live':True",p.read_text())
        self.assertIn("'compact_logs':True",p.read_text())


if __name__=='__main__':unittest.main()

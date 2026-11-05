import {afterEach,expect,it,vi} from 'vitest';
import {revealAcceptanceViews} from '../frontend/acceptanceViewport';
afterEach(()=>{document.body.innerHTML='';});
it('reveals the shared observation row only when diagnostics open',()=>{
  const row=document.createElement('div');row.className='e6-observation-row';
  row.scrollIntoView=vi.fn();document.body.append(row);
  const details=document.createElement('details');
  revealAcceptanceViews({currentTarget:details});expect(row.scrollIntoView).not.toHaveBeenCalled();
  details.open=true;revealAcceptanceViews({currentTarget:details});
  expect(row.scrollIntoView).toHaveBeenCalledWith({block:'start',behavior:'instant'});
});
it('does not need an observation row in isolated input fixtures',()=>{
  const details=document.createElement('details');details.open=true;
  expect(()=>revealAcceptanceViews({currentTarget:details})).not.toThrow();
});

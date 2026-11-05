import { useEffect } from 'react';
import useSWR from 'swr';
import type { ServerStatus } from '@/types';

// Discovery is read-only. Recovering it never prepares or replays control input.
export function useControlServerStatus() {
  const result = useSWR<ServerStatus>('e6-control/server-status', async () => {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 1500);
    try {
      const response = await fetch('/status', { signal: controller.signal });
      if (!response.ok) throw new Error(`Status HTTP ${response.status}`);
      const status = await response.json();
      if (!status || !Array.isArray(status.robot_status)) throw new Error('Invalid server status');
      return status;
    } finally {
      clearTimeout(timeout);
    }
  }, { refreshInterval: 5000, shouldRetryOnError: false, dedupingInterval: 1000 });
  const { error, isValidating, mutate } = result;
  useEffect(() => {
    if (!error || isValidating) return;
    const timer = setTimeout(() => { void mutate().catch(() => {}); }, 1000);
    return () => clearTimeout(timer);
  }, [error, isValidating, mutate]);
  return result;
}

export function ServerStatusRecovery({ retry, pending }: {
  retry: () => Promise<unknown>; pending: boolean;
}) {
  return <div role="status" className="rounded border p-3 space-y-2">
    <p>工作台服务暂不可用，正在等待恢复。连接恢复后需重新准备控制。</p>
    <button type="button" disabled={pending} onClick={() => { void retry().catch(() => {}); }}>
      {pending ? '正在检查连接…' : '立即重试连接'}
    </button>
  </div>;
}

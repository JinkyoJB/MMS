# main.py
"""
MMS 시스템 통합 테스트

실행 순서:
  1) MMS 초기화 (OrbbecClient warmup 포함)
  2) N_FRAMES 캡처
  3) Raw PCD 좌표 통계 출력
  4) Raw PCD 시각화 (ROI 전)
  5) 자동 ROI + 전처리 (voxel / denoise / normals)
  6) 전처리 결과 시각화
"""
from pathlib import Path

from mms.system import MMS, MMSConfig
from mms.sensor.orbbec_client import OrbbecConfig
from mms.utils.visualization import visualize
from mms.utils.diagnostics import print_pcd_stats, suggest_roi

ROOT = Path(__file__).parent

# ── 설정 ──────────────────────────────────────────────────────────────────
cfg = MMSConfig(
    orbbec=OrbbecConfig(
        sensor_frames_yaml=str(ROOT / "config/sensor_frames.yaml"),
        T_E_S_key="T_E_S_femto",
        enable_color=True,
        frame_timeout_ms=2000,
    )
)

N_FRAMES = 10


# ── main ──────────────────────────────────────────────────────────────────
def main() -> None:
    with MMS(cfg) as mms:
        # ── Step 1: 캡처 ──────────────────────────────────────────────────
        # 로봇 연결 시: batch = mms.capture_frames(N_FRAMES, ee_pose_fn=robot.get_ee_pose)
        batch = mms.capture_frames(N_FRAMES)
        if not batch:
            print("캡처된 프레임 없음. 종료.")
            return

        # ── Step 2: Raw 좌표 통계 (ROI bbox 결정용) ───────────────────────
        print_pcd_stats(batch)

        # ── Step 3: Raw PCD 시각화 (ROI 전) ──────────────────────────────
        # zero-depth 포인트는 capture 시점에 이미 필터링됨
        visualize("Step 3 — Raw PCD (ROI 전)", batch, max_pts_per_frame=150_000)


        # ── 누적 버퍼 확인 ────────────────────────────────────────────────
        print(f"\n[MMS stream 버퍼] 총 {len(mms.stream)} 프레임")


if __name__ == "__main__":
    main()

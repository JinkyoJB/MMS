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
import numpy as np

from mms import PROJECT_ROOT
from mms.system import MMS, MMSConfig
from mms.sensor.orbbec_client import OrbbecConfig
from mms.utils.visualization import visualize
from mms.utils.diagnostics import print_pcd_stats, suggest_roi

# ── 설정 ──────────────────────────────────────────────────────────────────
cfg = MMSConfig(
    orbbec=OrbbecConfig(
        sensor_frames_yaml=str(PROJECT_ROOT / "config/sensor_frames.yaml"),
        T_E_S_key="T_E_S_femto",
        enable_color=True,
        frame_timeout_ms=2000,
    ),
    object_frame_yaml=str(PROJECT_ROOT / "config/object_frame.yaml"),
)

N_FRAMES = 10


def verify_world_transform(mms: "MMS") -> None:
    """
    world_transform 동작 확인:
      - theta를 주면 T_O_B / T_B_O 변환행렬을 반환하는지
      - x_O → x_B → x_O 라운드트립이 정확한지
    """
    wt = mms.world_transform
    if wt is None:
        print("[world_transform] object_frame_yaml 미설정 — 스킵")
        return

    print("\n─── world_transform 검증 ───────────────────────────────────")
    print(f"T_B_O0 translation : {wt.T_B_O0[:3, 3]}")

    x_O = np.array([0.1, 0.0, 0.0, 1.0])   # 터닝테이블 위의 점 (O 좌표)

    for deg in [0, 45, 90, 180]:
        theta = np.radians(deg)
        x_B      = wt.T_O_B(theta) @ x_O          # O → B
        x_O_back = wt.T_B_O(theta) @ x_B          # B → O (역변환)
        err      = np.max(np.abs(x_O - x_O_back))
        print(f"  theta={deg:4d}°  x_B={x_B[:3]}  round-trip err={err:.2e}")

    print("────────────────────────────────────────────────────────────\n")


# ── main ──────────────────────────────────────────────────────────────────
def main() -> None:
    with MMS(cfg) as mms:
        # ── world_transform 검증 ──────────────────────────────────────────
        verify_world_transform(mms)

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

**1) 래퍼 메서드 정의**

| 래퍼 | 메서드 | 반환 타입 / numpy shape | 내부 SDK 타입(숨김) | 1차/2차 | 이유 |
| --- | --- | --- | --- | --- | --- |
| `ModelHandle` | `scans()` | `list[ScanHandle]` | `TRef<IModel>` → `IModel::getScan(i)` / `IScan`  | 1차 | 전체 결과에서 scan 단위로 내려가는 진입점. |
|  | `get_scan(i)` | `ScanHandle` | 같은 위 | 1차 | 특정 scan 접근. |
|  | `summary()` | `MeshSummary` | `IModel` + `ICompositeContainer` + `ICompositeMesh/IMesh`  | 1차 | vertex/face/texture/scan 개수 요약. |
|  | `save_obj(path)` | `None` | `ICompositeMesh` + Base IO (`saveObjCompositeToFile` 계열)  | 1차 | 최종 결과 export. |
|  | `scan_count()` | `int` | `IModel::getScanCount()` 또는 `getSize()`  | 1차 | scan 개수. |
|  | `has_final_mesh()` | `bool` | `IModel` + `ICompositeContainer::getSize()`  | 1차 | 최종 composite mesh 존재 여부. |
|  | `final_vertices()` | `np.ndarray`, shape `(N, 3)`, `float32` | `ICompositeMesh::getPoints()` → `IArrayPoint3F`  | 2차 | 최종 메시 geometry를 바로 numpy로. |
|  | `final_faces()` | `np.ndarray`, shape `(M, 3)`, `int32` | `ICompositeMesh::getTriangles()` → `IArrayIndexTriplet`  | 2차 | 최종 메시 faces를 바로 numpy로. |
| `ScanHandle` | `frame_count()` | `int` | `TRef<IScan>::getSize()`  | 1차 | scan 내 frame 수. |
|  | `frames()` | `list[FrameMeshHandle]` | `IScan::getElement(i)` → `IFrameMesh*`  | 2차 | 모든 frame 열람. |
|  | `get_frame(i)` | `FrameMeshHandle` | 같은 위 | 1차 | 특정 frame 접근. |
|  | `summary()` | `ScanSummary` | `IScan` (frame 개수 기준) [](https://.com/sdk/2.0/classartec_1_1sdk_1_1base_1_1_i_scan.html) | 1차 | scan 단위 요약. |
|  | `is_empty()` | `bool` | `IScan::getSize() == 0` [](https://.com/sdk/2.0/classartec_1_1sdk_1_1base_1_1_i_scan.html) | 2차 | 빈 scan 여부. |
|  | `last_frame()` | `FrameMeshHandle` | `IScan::getElement(size-1)`  | 2차 | 최근 frame 빠른 접근 (MMS에 유용). |
| `FrameMeshHandle` | `vertices()` | `np.ndarray`, shape `(N, 3)`, `float32` | `IFrameMesh::getPoints()` → `IArrayPoint3F`  | 1차 | 프레임 geometry. |
|  | `faces()` | `np.ndarray`, shape `(M, 3)`, `int32` | `IFrameMesh::getTriangles()` → `IArrayIndexTriplet`  | 1차 | 프레임 faces. |
|  | `is_textured()` | `bool` | `IFrameMesh::isTextured()`  | 1차 | texture 유무 판단. |
|  | `uv()` | `np.ndarray | None`, shape `(N, 2)`, `float32` | `IFrameMesh::getUVCoordinates()` → `IArrayUVCoordinates`  | 2차 | UV 좌표 접근. |
|  | `image()` | `np.ndarray | None`, shape `(H, W, C)`, `uint8` | `IFrameMesh::getImage()` → `IImage`  | 2차 | texture image (RGB). |
|  | `summary()` | `FrameMeshSummary` | `IFrameMesh` + `IMesh` + `IImage`  | 1차 | frame 단위 요약. |
|  | `has_image()` | `bool` | `IFrameMesh::getImage() != nullptr` [](https://.com/sdk/2.0/_i_frame_mesh_8h_source.html) | 2차 | texture 존재 여부. |
|  | `vertex_count()` | `int` | `IMesh::getPoints()->getSize()`  | 2차 | frame vertex 수. |
|  | `face_count()` | `int` | `IMesh::getTriangles()->getSize()`  | 2차 | frame face 수. |

**2) DTO 정의 (요약용)**

| DTO | 필드 | 타입 / 의미 | 내부 SDK 출처 |
| --- | --- | --- | --- |
| `MeshSummary` | `vertex_count` | `int` – 최종 메시 vertex 수 | `ICompositeMesh::getPoints()` [](https://.com/sdk/2.0/advanced.html) |
|  | `face_count` | `int` – 최종 메시 face 수 | `ICompositeMesh::getTriangles()` [](https://.com/sdk/2.0/advanced.html) |
|  | `textured` | `bool` – 텍스처 유무 | `ICompositeMesh::getTextureCount() > 0` [](https://.com/sdk/2.0/advanced.html) |
|  | `scan_count` | `int` – 포함된 scan 수 | `IModel::getScanCount()` [](https://.com/sdk/2.0/classartec_1_1sdk_1_1base_1_1_i_model.html) |
| `ScanSummary` | `frame_count` | `int` – scan 내 frame 수 | `IScan::getSize()` [](https://.com/sdk/2.0/classartec_1_1sdk_1_1base_1_1_i_scan.html) |
| `FrameMeshSummary` | `vertex_count` | `int` – frame vertex 수 | `IFrameMesh::getPoints()->getSize()`  |
|  | `face_count` | `int` – frame face 수 | `IFrameMesh::getTriangles()->getSize()`  |
|  | `textured` | `bool` – frame 텍스처 유무 | `IFrameMesh::isTextured()` [](https://.com/sdk/2.0/classartec_1_1sdk_1_1base_1_1_i_frame_mesh.html) |
|  | `image_width` | `int | None` – 텍스처 폭 | `IImage::getHeader().width` [](https://.com/sdk/2.0/_i_image_8h_source.html) |
|  | `image_height` | `int | None` – 텍스처 높이 | `IImage::getHeader().height` [](https://.com/sdk/2.0/_i_image_8h_source.html) |
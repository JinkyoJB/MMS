Project API는 Python에서 **`ProjectHandle` 중심 + 설정 DTO + 엔트리 DTO**로 가는 것이 가장 좋습니다. Algorithms API처럼 내부적으로는 job 기반이지만, Python에서는 가능한 한 **동기형 편의 메서드**로 보이게 하고, 내부에서 `createLoader()/createSaver()`와 `executeJob()`을 감추는 것이 사용성이 좋습니다. `IProject` 자체는 `IRef` 기반 객체이므로 역시 raw 인터페이스를 직접 노출하기보다 래퍼가 적합합니다.

## **Project API Python 래퍼 확정안 표**

| **래퍼** | **메서드** | **반환 타입 / shape** | **내부 SDK 타입(숨김)** | **1차/2차** | **이유** |
| --- | --- | --- | --- | --- | --- |
| `ProjectManager` | `create(path, settings=None)` | `ProjectHandle` | `createNewProject()` + `TRef<IProject>` + `ProjectSettings` | 1차 | 새 프로젝트 생성 진입점. |
|  | `open(path)` | `ProjectHandle` | `openProject()` + `TRef<IProject>` | 1차 | 기존 프로젝트 열기. |
|  | `max_supported_version()` | `int` | `getMaximumSupportedProjectVersion()` | 2차 | SDK 프로젝트 버전 호환성 확인. |
| `ProjectHandle` | `version()` | `int` | `IProject::getProjectVersion()` | 1차 | 현재 프로젝트 버전 확인. |
|  | `entry_count()` | `int` | `IProject::getEntryCount()` | 1차 | 프로젝트 엔트리 수 확인. |
|  | `entries()` | `list[ProjectEntryInfo]` | `IProject::getEntry()` + `EntryInfo` + `IString` | 1차 | 전체 엔트리 목록 조회. |
|  | `get_entry(index)` | `ProjectEntryInfo` | `IProject::getEntry()` | 1차 | 특정 엔트리 정보 조회. |
|  | `save(path=None, compression_level=None)` | `None` | `IProject::createSaver()` + `ProjectSaverSettings` + `IJob` | 1차 | 현재 프로젝트 저장. Python에서는 동기 메서드로 감싼다. |
|  | `load_entries(settings=None)` | `list[LoadedProjectEntry]` | `IProject::createLoader()` + `ProjectLoaderSettings` + `IJob` | 1차 | 프로젝트 내부 scan/model 엔트리 실제 로드. |
|  | `copy_to(path, settings=None)` | `ProjectHandle` 또는 `None` | `IProject::createCopier()` + `ProjectCopierSettings` + `IJob` | 2차 | 프로젝트 복사. |
|  | `delete(settings=None)` | `None` | `IProject::createDeleter()` + `ProjectDeleterSettings` + `IJob` | 2차 | 문서상 미구현 가능성 있어 2차. |
| `ProjectEntryInfo` | `entry_id` | `str`(UUID) | `EntryInfo.uuid` | 1차 | 프로젝트 내 엔트리 식별자. |
|  | `entry_type` | `EntryType` enum | `EntryInfo.type` | 1차 | scan/model 타입 구분. |
|  | `name` | `str` | `IString** entryName` | 1차 | 엔트리 이름. |
| `LoadedProjectEntry` | `entry_info` | `ProjectEntryInfo` | `EntryInfo` | 1차 | 로드된 엔트리의 메타데이터. |
|  | `scan` | `ScanHandle \| None` | project loader가 반환한 `IScan` | 1차 | scan 엔트리인 경우 Base 래퍼로 연결. |
|  | `model` | `ModelHandle \| None` | project loader가 반환한 `ICompositeMesh`/model 관련 데이터 | 1차 | model 엔트리인 경우 Base 래퍼로 연결. |
| `ProjectSettingsDTO` | `path` | `str` | `ProjectSettings.path` | 1차 | 새 프로젝트 생성 위치. |
| `ProjectSaverSettingsDTO` | `path`, `project_id`, `compression_level` | `str`, `str`, `int` | `ProjectSaverSettings` | 1차 | 저장 옵션. |
| `ProjectLoaderSettingsDTO` | 로드 정책 필드들 | bool/int/enum | `ProjectLoaderSettings` | 2차 | loaded/unloaded/key frames only 등 세부 정책 반영용. |
| `ProjectCopySettingsDTO` | 복사 관련 필드 | path 등 | `ProjectCopierSettings` | 2차 | 프로젝트 복사 옵션. |

## **Python enum/DTO 표**

| **Python 타입** | **SDK 출처** | **값 / 의미** |
| --- | --- | --- |

| **Python 타입** | **SDK 출처** | **값 / 의미** |
| --- | --- | --- |
| `EntryType` | `EntryType` | `SCAN`, `COMPOSITE_MESH`, `UNKNOWN` |
| `ProjectEntryInfo` | `EntryInfo` | `entry_id`, `entry_type`, `name` |
| `LoadedProjectEntry` | loader 결과 + `EntryInfo` | Python 편의 DTO. scan 또는 model 중 하나를 담음. |
| `ProjectSettingsDTO` | `ProjectSettings` | 프로젝트 생성용 path 등 |
| `ProjectSaverSettingsDTO` | `ProjectSaverSettings` | path, project_id, compression_level |

---

## **shape 관련 메모**

Project API는 Base처럼 numpy shape를 직접 다루는 레이어가 아닙니다. 대부분의 메서드는 **프로젝트 핸들, 엔트리 DTO, 또는 Base 래퍼(`ScanHandle`, `ModelHandle`)**를 반환하는 것이 맞습니다. 실제 geometry shape `(N,3)` / `(M,3)`는 Project API가 아니라, 프로젝트에서 load한 후 Base API 래퍼를 통해 꺼내는 구조가 자연스럽습니다.

즉 shape는 이렇게 연결됩니다.

| **Project API 결과** | **shape 접근 위치** |
| --- | --- |
| `LoadedProjectEntry.scan` | `ScanHandle.get_frame(i).vertices()` → `(N,3)` |
| `LoadedProjectEntry.model` | `ModelHandle.final_vertices()` → `(N,3)` |
| `LoadedProjectEntry.model` | `ModelHandle.final_faces()` → `(M,3)` |
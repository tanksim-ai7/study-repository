# study-repository
git test 용도 입니다.

#STUDY, TEST 용도

## Git 운영 규칙

- 아래 규칙을 꼭 확인하시고 지켜주시면 감사하겠습니다.
- 문제가 발생했을 때는 git 담당자에게 문의 주시면 감사하겠습니다.

### 1. Branch 생성 규칙
- 각 조별 main branch에서 개인별 branch를 추가하여 작업할 것
- origin/main에는 직접 새로운 branch를 만들지 말 것
- 조별 main branch에 병합 완료된 개인 branch는 삭제할 것

### 2. Commit 규칙
- 모든 코드는 commit 전 아래 내용을 확인할 것
  1) 코드에 오류가 없는지 확인할 것 (AI를 활용한 코드 검증 권장)
  2) Commit 시에는 항상 커밋 메시지를 작성할 것
     - 첫 줄: 추가/수정한 기능에 대한 핵심 요약
     - 다음 줄부터: 추가/수정한 내용에 대한 상세 사항 기입
       (다른 사람이 커밋 메시지만 보고도 내용을 파악할 수 있도록 작성)
  3) 개인 branch에서 작업한 내용을 조별 main branch에 병합하기 전, 각 조 git 담당자의 허가를 받을 것

### 3. origin/main 병합 규칙
origin/main에 병합하기 전 아래 과정을 반드시 거칠 것
  1) 코드 내용에 오류가 없는지 최종 확인할 것 (AI 사용 권장) — 자잘한 오류가 쌓이면 추후 문제가 커질 수 있음
  2) 조별 main branch에 origin/main으로 병합할 코드 내용을 올려둘 것
  3) 조별 main branch(작업 branch)로 이동한 상태에서 origin/main을 병합하여 충돌 여부를 먼저 확인할 것
     - 충돌이 있다면 해결 후 새로운 commit을 생성
  4) 조별 main branch → origin/main은 Pull Request를 통해 진행하며, git 담당자 확인 후 병합한다

### 4. Branch 네이밍 규칙
각 조별 main branch:
- 자율주행팀: `auto/main`
- 오브젝트 디텍션팀: `detect/main`
- 터렛 제어팀: `turret/main`

개인 작업 branch는 조별 main branch 하위에 작업 내용을 알 수 있는 이름으로 생성:
```
main                (건들지 않음, 최종 병합 대상)
├─ auto/main        ← 자율주행팀 조별 main branch
│   └─ auto/a_star  ← 개인 작업 branch
├─ detect/main       ← 오브젝트 디텍션팀 조별 main branch
│   └─ detect/YOLO   ← 개인 작업 branch
└─ turret/main       ← 터렛 제어팀 조별 main branch
    └─ turret/Calc   ← 개인 작업 branch
```

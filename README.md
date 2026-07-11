# roscamp-repo-3
ROS2와 AI를 활용한 자율주행 로봇개발자 부트캠프 3팀 저장소. WaSaB (와사비) 



## System Architecture

<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/a7a8af14-099d-4407-994c-12908d8708c5" width="1000"></td>
    <td><img src="https://github.com/user-attachments/assets/cf6757ce-d1a7-48f0-901e-e5724302b3a4" width="1000"></td>
  </tr>
  <tr>
    <td align="center">[ HW Architecture ]</td>
    <td align="center">[ SW Architecture ]</td>
  </tr>
</table>


## GitHub 컨벤션

### 브랜치
- **`main`** : 배포용 브랜치
- **`develop`** : 개발용 브랜치
- **`Feat`** : 개인 개발용 브랜치

### 라벨링
- **`Feat`** : 기능 구현을 필요로 할 때 사용
- **`Fix`** : 버그 수정을 필요로 할 때 사용
- **`Chore`** : 잡다한 수정을 필요로 할 때 사용 (디자인 리소스 추가, 주석, 코드 리포맷 등)

### 메시지
- **`Issue`**
  - [Label] Issue Title
    - ex1) [Feat] 추종 기능 구현
    - ex2) [Fix] 주행 파라미터 수정
- **`Branch`**
  - 기능 별 브랜치 생성 시 다 쓰면 삭제
  - Label/IssueNumber-BranchName
    - ex1) Fix/#023-EdtSomething
    - ex2) Chore/#123-AddMap
- **`Commit`**
  - [IssueNumber] Commit Message
    - ex1) [#203] 추종 알고리즘 구현
    - ex2) [#207] 주행 파라미터 통합
- **`PR`**
  - [Label/IssueNumber] Issue Title
    - ex1) [Feat/#037] 추종 기능 구현
    - ex2) [Chore/#099] 지도 추가


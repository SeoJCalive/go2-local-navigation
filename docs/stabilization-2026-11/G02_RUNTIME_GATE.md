<!-- 목적: built runtime의 구현 일치·G02 합격 여부와 정상 loop 손실을 조회한다.
흐름/입출력: 고정 source·raw scan·qpos·실제 graph → 재생/GUI/거부 전체 판정.
주의: 사후 GT는 runtime 입력이 아니며, simulation 합격과 실물 검증을 구분한다. -->
# G02 runtime odometry-distance gate 검증

2026-11-26 KST 실행이다. runtime 구현은 지정된 거리 gate와 일치하지만,
G02 attempt 9는 **FAIL**이다. simulation 지도 완료로 판정하지 않는다.

## 구현과 경계

- branch: `feat/loop-gate-runtime`, 시작점 `b313e92085191c1984a9532910b4f27c0f4a5108`.
- upstream: slam_toolbox 2.6.10, `8293d21fe5c816d0405e0cc1f3eb46c70659c267`.
- parameter: `loop_closure_max_odom_distance`, double, 기본 0.0=비활성.
- linked anchor는 fine 제안의 barycenter에서 upstream과 같은 방식으로 선택한다.
- 원본 두 base odometry의 XY 거리가 1.0 m를 초과하면 pose·edge·solve 변경 전에 거부한다.
- simulation 두 공개 진입점만 1.0을 전달한다. workspace source 순서로 private overlay를 선택하며 잘못된 prefix를 거부한다.
- 실물/shared `slam_mapping.yaml`은 변경하지 않았다. SHA256은 `adc6456037cc5448d7f4a8237026b57c7a9b7f895e7e8341de68f5d7143b1529`이다.
- private prefix는 `third_party/slam_toolbox/install/slam_toolbox`이다. 시스템 설치를 대체하지 않는다.
- GUI의 실제 `/proc/maps`에서 SDK/common/Ceres plugin 세 library가 모두 이 prefix임을 확인했다.
- raw gate field는 기존 posegraph archive 순서에 추가하지 않았다. 저장 graph를 system 2.6.10 reader로 읽어 5,410개 immutable factor를 추출했다.

## GUI 이전 offline 동등성

- built runtime SDK/common으로 12 dataset × parameter 0/1 m = 24회 모두 통과했다.
- 0 m는 기존 `none`, 1 m는 기존 `distance-1` 결과와 일치했다.
- edge/factor/online/final vertices/PGM 다섯 출력은 byte 단위로 일치했다. 요약 지표 허용 오차는 `1e-8`이다.
- 1 m의 strict wrong loop는 12개 모두 0이며 correct loop 합계는 21이다.
- G02 attempt-6 입력의 offline 1 m 결과는 wall offset p95 0.057099819850 m, doubled 6, ghost 0, 최대 pose 오차 0.269156524005 m였다.
- 이 구현 일치는 실제 GUI의 지도 정확도 합격을 뜻하지 않는다.
- 전체 project 10 package build와 CI 동일 paths/flags의 `rosdep install --simulate`는 exit 0이었다.
- rosdep은 사용자 source/cache만 초기화했다. 최초 custom-cache 지정 실패는 실제 기본 cache를 사용해 바로잡았다.

## 실제 GUI attempt 9

- route SHA256: `8b9cbeb85a44c795137447e8350b46aadba1fd21288497dbb1ec12a88207246e`.
- 실행시간: 1670.602141 wall s. 같은 275.372560 m route를 완주했다.
- 실제 query parameter는 거리 1.0, coarse variance 3.0, coarse response .35, fine response .45, chain 10, search dimension 8, max distance 3이었다.
- 독립 driver 목표 period는 .020 s, 실측 p95는 .020887313 s, 최대 간격은 .278386510 s였다.
- 최초 0.3 m 경보 이전 driver 최대 간격은 .040316773 s였다.
- 전체 최대 간격은 1545.566 s에서 발생했고, 해당 iteration의 status write가 .277164093 s였다. 최초 pose 경보는 그보다 앞선 746.596 s였다.
- contact는 150개, 11 episode, 최대 깊이 .000514143 m, 최대 episode .044 s, deep-contact 지속 0 s였다. 기존 중단 정책을 유지했다.
- 비차단 capture와 한국어 `live/progress.log`를 유지했고 실제 MuJoCo/RViz와 최종 GT overlay를 직접 확인했다.
- GUI 종료 결과는 owned residue 0, 원본 simulator config 유지, 기존 process identity 유지였다.
- attempt 8은 새 native library 검사의 wrapper/native 중복 집계로 주행 전에 중단됐다. `argv[0]`로 고치고 회귀 검사를 추가한 뒤 attempt 9를 수행했다.

| 지표 | 어제 attempt 6 | runtime attempt 9 | 기존 기준/결과 |
| --- | ---: | ---: | --- |
| occupied wall offset p95 m | 1.062680816232 | 0.075041032795 | 비교 지표 |
| normal profile offset p95 m | 0.186250000000 | 0.073750000000 | ≤.05, FAIL |
| normal profile offset max m | 0.198750000000 | 0.106250000000 | ≤.15, PASS |
| doubled wall component | 9 | 8 | 0, FAIL |
| ghost component | 75 | 0 | 0, PASS |
| profile thickness p95 m | 0.350000000000 | 0.15000000000000013 | ≤.15, 엄격 비교 FAIL |
| pose max m | 5.027769507804 | 0.394611847569 | 기존 .10·추가 alert .30 모두 초과 |
| pose p95 m | 4.404264418372 | 0.159595246786 | 비교 지표 |

이중벽은 north_band_west 1, south_band_west 2, south_band_mid 3,
south_band_east 1, west_north_room_wall 1로 합계 8이다. 기준은 바꾸지 않았다.

## (a) 이번 run의 실제 수락과 거리 거부

- native accepted closure event: 8회.
- 실제 신규 loop edge: 8개. 저장 immutable factor/GT 판정은 correct 8, wrong 0이다.
- `LOOP_ODOM_GATE_REJECT`: 84회. 기존 coarse/fine matcher 거부와 구분한 숫자다.
- 거부 candidate의 GT 판정: 정상 76, 오류 8이다.
- 정상 기준은 기존과 동일하게 상대 계측의 XY ≤.10 m 및 yaw ≤5°이다.
- 거부된 factor는 생성되지 않으므로 recorded chain anchor/fine pose의 would-be 상대 계측을 독립 qpos GT와 비교했다. 실제 수락은 저장 LinkInfo를 사용했다.
- 아래 표는 84회 전체다. 표시 반올림 전 값은 `final-audit/loop-audit.json`에 보존했다.

| sim s | from→to | odom m | GT 정상 | 상대 XY m | 상대 yaw deg | 기존 절대 m | fine 제안 절대 m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 393.906 | 142→278 | 2.324566146 | False | 3.560755 | 0.523239 | 0.120635 | 3.607482 |
| 418.904 | 236→316 | 2.628165548 | False | 0.163122 | 0.671962 | 0.135756 | 0.088831 |
| 419.406 | 236→317 | 2.544838858 | False | 0.162857 | 0.467768 | 0.135598 | 0.085767 |
| 422.104 | 236→320 | 2.194430865 | True | 0.014152 | 0.250207 | 0.135279 | 0.090741 |
| 429.804 | 241→328 | 1.001617536 | True | 0.009862 | 0.126750 | 0.104186 | 0.087506 |
| 558.300 | 467→528 | 4.417815837 | True | 0.057622 | 0.582261 | 0.060030 | 0.082872 |
| 560.300 | 467→531 | 4.336945472 | True | 0.036128 | 0.178849 | 0.051843 | 0.075987 |
| 560.700 | 467→532 | 4.323818568 | True | 0.036881 | 0.022834 | 0.050171 | 0.073102 |
| 561.202 | 467→533 | 4.316056577 | True | 0.035471 | 0.018907 | 0.053641 | 0.078753 |
| 644.196 | 7→666 | 2.969862091 | True | 0.027133 | 0.356270 | 0.085030 | 0.015539 |
| 645.498 | 7→668 | 3.022196605 | True | 0.029003 | 0.352571 | 0.089458 | 0.014816 |
| 646.300 | 7→669 | 3.080955175 | True | 0.010854 | 0.146756 | 0.092097 | 0.006927 |
| 657.198 | 11→686 | 1.655118620 | True | 0.005358 | 0.169112 | 0.105447 | 0.000880 |
| 777.696 | 70→859 | 1.465179614 | True | 0.019075 | 0.218228 | 0.241618 | 0.031481 |
| 780.992 | 70→864 | 1.035383913 | True | 0.004884 | 0.038448 | 0.216648 | 0.036346 |
| 793.996 | 787→877 | 2.089457415 | True | 0.026445 | 0.354103 | 0.032766 | 0.038704 |
| 794.792 | 787→879 | 1.970520960 | True | 0.013652 | 0.014817 | 0.029255 | 0.022684 |
| 795.596 | 784→880 | 2.117481596 | True | 0.021838 | 0.250384 | 0.026254 | 0.021643 |
| 796.092 | 784→881 | 2.060037599 | True | 0.030150 | 0.264497 | 0.024472 | 0.031069 |
| 796.494 | 784→882 | 2.010579898 | True | 0.019799 | 0.464311 | 0.022251 | 0.020494 |
| 799.292 | 787→884 | 1.637179214 | True | 0.014375 | 0.328048 | 0.016439 | 0.021508 |
| 800.294 | 787→885 | 1.522713392 | True | 0.015228 | 0.123453 | 0.025211 | 0.023808 |
| 800.692 | 787→886 | 1.425173223 | True | 0.016229 | 0.353872 | 0.025672 | 0.026157 |
| 801.096 | 787→887 | 1.333400231 | True | 0.016913 | 0.370405 | 0.026379 | 0.028139 |
| 802.194 | 787→888 | 1.164250802 | True | 0.017518 | 0.730657 | 0.028081 | 0.031190 |
| 803.892 | 787→889 | 1.147774202 | True | 0.008359 | 0.523538 | 0.029442 | 0.022445 |
| 807.988 | 787→893 | 1.355245371 | True | 0.013742 | 0.650021 | 0.035437 | 0.029092 |
| 808.396 | 787→894 | 1.296312414 | True | 0.016658 | 0.051571 | 0.035798 | 0.033295 |
| 808.796 | 787→895 | 1.211339906 | True | 0.010244 | 0.011622 | 0.036045 | 0.026258 |
| 809.194 | 786→896 | 1.257976265 | True | 0.019329 | 0.150192 | 0.036825 | 0.024451 |
| 809.592 | 787→897 | 1.100423854 | True | 0.018399 | 0.008871 | 0.037866 | 0.039005 |
| 883.794 | 687→1012 | 1.857156645 | True | 0.002777 | 0.032762 | 0.028021 | 0.072440 |
| 937.490 | 4→1101 | 2.423157210 | True | 0.001772 | 0.273226 | 0.010178 | 0.003576 |
| 937.490 | 658→1101 | 3.426382886 | True | 0.021793 | 0.148927 | 0.010178 | 0.052620 |
| 937.490 | 1035→1101 | 3.055656185 | True | 0.009557 | 0.241133 | 0.010178 | 0.005761 |
| 938.292 | 4→1102 | 2.480851969 | True | 0.002651 | 0.276538 | 0.015338 | 0.002064 |
| 938.292 | 1035→1102 | 3.173577964 | True | 0.013921 | 0.144464 | 0.015338 | 0.015338 |
| 940.092 | 4→1105 | 2.657686624 | True | 0.005273 | 0.279371 | 0.009603 | 0.008997 |
| 940.092 | 1035→1105 | 3.456536002 | True | 0.011941 | 0.052665 | 0.009603 | 0.013112 |
| 943.694 | 1035→1110 | 4.088072030 | True | 0.018698 | 0.159382 | 0.010306 | 0.016886 |
| 944.494 | 657→1111 | 2.569957764 | True | 0.004326 | 0.171929 | 0.009980 | 0.057976 |
| 944.986 | 657→1112 | 2.537061904 | True | 0.004899 | 0.031575 | 0.014889 | 0.057286 |
| 949.492 | 1035→1117 | 4.686671785 | True | 0.012291 | 0.196872 | 0.009023 | 0.009023 |
| 949.892 | 657→1118 | 2.114993235 | True | 0.000761 | 0.208582 | 0.008110 | 0.057856 |
| 949.892 | 1035→1118 | 4.625611347 | True | 0.019448 | 0.198449 | 0.008110 | 0.006111 |
| 950.294 | 657→1119 | 2.005390812 | True | 0.000966 | 0.026546 | 0.007618 | 0.057494 |
| 950.294 | 1035→1119 | 4.567975783 | True | 0.013069 | 0.163282 | 0.007618 | 0.007618 |
| 950.692 | 657→1120 | 1.894978962 | True | 0.009420 | 0.156973 | 0.016014 | 0.063853 |
| 950.692 | 1035→1120 | 4.518775729 | True | 0.013189 | 0.146840 | 0.016014 | 0.007284 |
| 957.990 | 651→1129 | 1.270698143 | True | 0.003294 | 0.178934 | 0.020460 | 0.047965 |
| 958.390 | 566→1130 | 2.430853575 | True | 0.017810 | 0.105671 | 0.020350 | 0.047414 |
| 958.792 | 566→1131 | 2.345349333 | True | 0.017184 | 0.117830 | 0.020283 | 0.047108 |
| 959.692 | 565→1132 | 2.177810495 | True | 0.004239 | 0.140138 | 0.020609 | 0.046502 |
| 960.192 | 565→1133 | 2.071595222 | True | 0.001823 | 0.069681 | 0.020716 | 0.044638 |
| 960.992 | 564→1134 | 1.996756746 | True | 0.019655 | 0.370662 | 0.020856 | 0.046630 |
| 960.992 | 652→1134 | 1.361555463 | True | 0.019899 | 0.138944 | 0.020856 | 0.043113 |
| 961.392 | 564→1135 | 1.932380992 | True | 0.019548 | 0.353793 | 0.021308 | 0.046929 |
| 961.392 | 652→1135 | 1.464151755 | True | 0.020136 | 0.122075 | 0.021308 | 0.043334 |
| 961.782 | 564→1136 | 1.864315087 | True | 0.019511 | 0.532469 | 0.021809 | 0.047104 |
| 961.782 | 652→1136 | 1.564474521 | True | 0.010333 | 0.100789 | 0.021809 | 0.047104 |
| 962.494 | 561→1137 | 1.929942924 | True | 0.018843 | 0.391554 | 0.022763 | 0.047589 |
| 962.494 | 653→1137 | 1.651516468 | True | 0.004894 | 0.120602 | 0.022763 | 0.047589 |
| 963.292 | 653→1138 | 1.841111330 | True | 0.004451 | 0.332175 | 0.023765 | 0.048131 |
| 965.092 | 627→1140 | 4.084660410 | True | 0.027242 | 0.010066 | 0.016095 | 0.044823 |
| 968.692 | 561→1143 | 1.329690831 | True | 0.016983 | 0.263427 | 0.019079 | 0.050067 |
| 968.692 | 652→1143 | 2.675275159 | True | 0.020371 | 0.237231 | 0.019079 | 0.045005 |
| 969.074 | 562→1144 | 1.187251415 | True | 0.015510 | 0.127329 | 0.019189 | 0.049512 |
| 969.390 | 562→1145 | 1.102650498 | True | 0.014958 | 0.117116 | 0.019027 | 0.049050 |
| 969.790 | 562→1146 | 1.011020361 | True | 0.005314 | 0.285294 | 0.018961 | 0.057077 |
| 977.588 | 465→1160 | 1.134951263 | True | 0.009816 | 0.347103 | 0.046684 | 0.060587 |
| 977.992 | 464→1161 | 1.116724632 | True | 0.011095 | 0.108130 | 0.054007 | 0.066536 |
| 1089.486 | 7→1331 | 2.415340802 | True | 0.014857 | 0.112787 | 0.042199 | 0.000687 |
| 1089.486 | 1040→1331 | 3.174546130 | True | 0.025935 | 0.104309 | 0.042199 | 0.013925 |
| 1197.788 | 1387→1483 | 1.860651569 | True | 0.029190 | 0.714060 | 0.100443 | 0.100019 |
| 1197.788 | 1420→1483 | 1.507315213 | True | 0.005946 | 0.646807 | 0.100443 | 0.095116 |
| 1198.290 | 1387→1484 | 1.804361558 | True | 0.021588 | 0.721639 | 0.099402 | 0.090108 |
| 1198.290 | 1420→1484 | 1.509267771 | True | 0.005839 | 0.239304 | 0.099402 | 0.093809 |
| 1198.688 | 1387→1485 | 1.748937816 | True | 0.019686 | 0.516096 | 0.098020 | 0.088198 |
| 1198.688 | 1420→1485 | 1.511345007 | True | 0.006002 | 0.844771 | 0.098020 | 0.092159 |
| 1372.286 | 281→1738 | 2.542737932 | False | 3.572796 | 0.151381 | 0.042483 | 3.557760 |
| 1372.286 | 354→1738 | 2.855357792 | False | 3.576911 | 0.082411 | 0.042483 | 3.557760 |
| 1376.286 | 287→1745 | 2.777544450 | False | 3.557099 | 0.500375 | 0.046530 | 3.542880 |
| 1376.286 | 343→1745 | 3.273031909 | False | 3.562325 | 0.583376 | 0.046530 | 3.542880 |
| 1391.186 | 276→1774 | 2.157509904 | False | 3.561935 | 0.099644 | 0.040318 | 3.550685 |


## (b) 정상 loop 손실과 drift

- 정상 거부 76개 중 31개의 fine 제안은 당시 절대 GT 오차를 줄였다.
- 최초 경보 746.596 s 이전에는 거리 거부 13개 중 GT 정상 10개, 그중 즉시 오차 감소 제안 6개였다.
- 646.300 s의 `7→669`는 odometry 거리 3.080955174865 m로 거부됐다. 상대 오차는 .010854129 m·.146756447°로 정상이다.
- 이 후보의 기존 절대 오차 .092097315 m에 비해 fine 제안은 .006926778 m였다.
- 고정 1 m는 서로 떨어진 base node의 정상 중첩 scan constraint도 차단한다. Karto의 barycenter anchor 선택에서는 linked node 원점이 1 m 이내일 필요가 없다.

이 관찰은 정상 fine 후보를 실제로 차단했다는 근거다. 후보의 즉시 제안 오차만으로
신규 constraint 생성과 최적화 후 전체 경로 개선을 확정하지 않는다.

upstream은 fine 통과 뒤에도 barycenter 간 거리 1.5 m를 검사한다.
`7→669`만 예외 허용한 worker1 재생에서는 gate를 한 번 통과했지만
해당 edge를 만들지 않았다. `CorrectPoses()`는 이 경우에도 호출된다.

| 같은 고정 raw 입력 | 거리 1 m 기준 | `7→669` fine 예외 |
| --- | ---: | ---: |
| 처리 node | 2,092 | 2,092 |
| accepted closure event | 8 | 9 |
| 신규 loop edge | 8 | 8 |
| GT wrong loop | 0 | 0 |
| 746.596 s pose 오차 m | .302522946604 | .363022407497 |
| 전체 최대 pose 오차 m | .394611847569 | .454077025113 |
| 최초 .3 m 초과 s | 746.596 | 739.890 |

기준 재생은 실제 GUI의 최대·p95·경보 시점 오차를 재현했다.
이 예외는 신규 정상 constraint의 효과를 검증한 사례로 해석할 수 없다.
사후 fine 제안만으로 “이 loop가 drift를 고쳤을 것”이라고 주장하지 않는다.

429.804 s의 정상 후보 `241→328`도 별도로 대조했다.
odometry 거리 1.001617535680 m, 상대 GT 오차 .009862358 m·.126750466°이며,
기존 절대 오차 .104185980 m에 대한 fine 제안은 .087506358 m였다.
이 예외는 실제 `241→328` edge를 만들었다.

| 같은 고정 raw 입력 | 거리 1 m 기준 | `241→328`만 예외 |
| --- | ---: | ---: |
| 처리 node | 2,092 | 2,092 |
| accepted closure event / 신규 loop edge | 8 / 8 | 10 / 10 |
| 실제 edge의 GT correct / wrong | 8 / 0 | 10 / 0 |
| 746.596 s pose 오차 m | .302522946604 | .459822093131 |
| 전체 최대 pose 오차 m | .394611847569 | .547831248527 |
| pose p95 m | .159595246786 | .260941424666 |
| 최초 .3 m 초과 s | 746.596 | 718.998 |

정상 constraint를 실제 차단한 사례는 확인했다. 그러나 이 constraint를 복원하면
이번 drift가 줄어든다는 주장은 이 대조 결과와 맞지 않는다. fine 제안의 국소 오차와
전체 graph 최적화 뒤 trajectory 오차가 다르다.

- 두 예외는 사후 GT로 선별한 진단 전용이며 runtime gate·GUI 기준에는 반영하지 않았다.
- 나머지 정상 후보 74개의 실제 edge 생성 여부와 재주입 인과 효과는 검증하지 않았다.
- 첫 오차 증가는 accepted loop가 없는 동안 발생했다. 이를 false accepted loop에 의한 merge로 판정할 근거는 없다.
- 정상 후보 손실만으로 해당 drift의 원인을 확정하지 않는다. backend drift의 세부 원인은 미확정이다.
- 재현 source·결과: `.omo/evidence/loop-gate-runtime/diagnostic-source/`, `diagnostic/`, `measure_diagnostic.py`.

## (c) no last accepted edge

- 최초 경보: 746.596 s, 오차 .302522946604 m, 마지막 수락 edge는 null.
- 이번 run의 최초 실제 수락은 781.876 s의 `70→865`다. 최초 경보 시점에는 수락 loop가 없었다.
- live reader는 현재 run의 파일을 처음부터 읽는다. observer 시작 이후 행만 읽는 방식이 아니다.
- 해당 필드는 최초 경보의 고정 snapshot이다. 이후 수락을 과거 경보에 소급하지 않는다.
- fresh mapping이므로 이전 attempt의 edge를 합치지 않는 것이 맞다. `prepare_run`은 producer보다 먼저 실행했다.
- 별도 offline replay reader는 native 종료 시 CSV 마지막 미완성 행 `1`을 처리하지 못했다. 기존 `Lines` framer 재사용과 회귀 test로 수정했고 raw CSV는 보존했다.
- 수정 후 독립 replay가 최초 시각·오차·null edge와 전체 최대 .394611847569 m를 그대로 재현했다.

## Acceptance 1회

- 실제 runtime overlay를 source한 `acceptance.py --domain 41` 실행은 exit 0, PASS다.
- 원래 criteria 12개가 모두 true다. mapping route 완료·stall 0·nonfloor contact 0·map save·reset recovery를 통과했다.
- goal 5개 모두 `SUCCEEDED`이며 GT 위치 오차는 .164042124, .127244745, .133594357, .134027859, .167925806 m다.
- 최대 goal 오차 .167925806 m는 기존 .3 m 기준 이내다.
- owned residue·cleanup error·shutdown error·SIGKILL·escalation은 0이고 기존 process identity를 보존했다.
- mapping 시간은 224.657036 wall s였다.
- epoch reset 중 기존 TF2의 `TF_OLD_DATA` 경고가 stdout에 남았다. reset recovery criterion은 통과했고 경고를 제거하거나 숨기지 않았다.
- 결과: `.omo/evidence/loop-gate-runtime/acceptance/result.json`, 원본 stdout `acceptance.log`.

## 전체 build/test

- private SLAM build와 project 10-package build는 모두 exit 0이다.
- CI와 같은 `src optional/real_robot tools/go2_validation` 경로의 `rosdep install --simulate --ignore-src --rosdistro humble --as-root apt:false --reinstall -y`는 exit 0이다.
- 전체 `colcon test`는 한 번 수행했다. 10개 package 완료, 8분 27초, test/result exit 0이다.
- colcon 집계는 355 tests, errors 0, failures 0, skipped 2다.
- skip 2개는 변경하지 않은 `optional/real_robot/{bringup,go2_sensor}/test/test_copyright.py`의 기존 generated-source skip이다.
- 같은 두 package의 flake8 test는 기존 `SelectableGroups dict interface is deprecated. Use select.` warning을 냈다. warning/skip은 숨기거나 바꾸지 않았다.
- 새 CSV framing·native PID·runtime parameter 회귀를 포함한 나머지 검사도 통과했다.
- 원본: `.omo/evidence/loop-gate-runtime/full-colcon-test.log`, `build/*/test_results/`, `build/{bringup,go2_sensor,go2_validation}/pytest.xml`.

```bash
colcon test --base-paths src optional/real_robot tools/go2_validation \
  --executor sequential --return-code-on-test-failure \
  --event-handlers console_direct+ --ctest-args " -j1"
colcon test-result --verbose
```

## 증거

- owner live: `.omo/evidence/g02-wall-hug/live/`, `progress.log`, `final-map-ground-truth-overlay.png`.
- 동결 raw: `.omo/evidence/loop-gate-runtime/g02-attempt9/`, 22개 입력 SHA manifest 포함.
- final Dataset SHA256: `8b6d0d6e8779fd35207463dfcc5e8a21d85e58c236264caa43a3cb4d8fe2d854`.
- final posegraph SHA256: `ed772c6f5494c09b189386728c1d489d5800931e0f111e966a68cfb909d945a9`.
- 구현 동등성: `.omo/evidence/loop-gate-runtime/equivalence/equivalence.json`.
- 수락/거부 전체 판정: `.omo/evidence/loop-gate-runtime/final-audit/loop-audit.{json,md}`.
- 분석 재현: 같은 경로의 `analyze_loop_audit.py`; raw frame 보존·기존 상대 GT classifier 사용.

## 변경 파일

| 파일 | 역할 |
| --- | --- |
| `third_party/slam_toolbox/COLCON_IGNORE` | private source/build를 project 탐색에서 분리 |
| `third_party/slam_toolbox/setup.sh` | 2.6.10 pin·patch·private build와 receipt |
| `third_party/slam_toolbox/runtime-gate.patch` | double 기본 0, fine 수용점 odometry 거리 거부와 로그 |
| `third_party/slam_toolbox/README.md` | 한국어 source 순서·재현·결과 경계 |
| `.gitignore` | overlay source/build/install/log 제외 |
| `src/go2_mujoco_lidar/launch/go2_sim_navigation.launch.py` | overlay prefix 확인과 simulation gate 1.0 |
| `src/go2_mujoco_lidar/launch/go2_sim_bringup.launch.py` | 같은 simulation 진입점으로 mapping 위임 |
| `src/go2_mujoco_lidar/CMakeLists.txt` | runtime 회귀 test 등록 |
| `src/go2_mujoco_lidar/test/wall_hug/counterfactual/CMakeLists.txt` | 기존 standalone/built-runtime 이중 build |
| `src/go2_mujoco_lidar/test/wall_hug/counterfactual/engine_replay.cpp` | 실제 built SDK parameter·version 확인 |
| `src/go2_mujoco_lidar/test/wall_hug/runtime_equivalence.py` | 12개 입력 × 0/1 m byte·지표 비교 |
| `src/go2_mujoco_lidar/test/wall_hug/runtime_gui.py` | 같은 GUI·route·driver·capture·정책을 runtime overlay로 실행 |
| `src/go2_mujoco_lidar/test/test_wall_hug_runtime.py` | parameter 경계·비교기·native PID 회귀 |
| `src/go2_mujoco_lidar/test/wall_hug/pose_watch.py` | offline CSV도 기존 complete-line framer 사용 |
| `src/go2_mujoco_lidar/test/test_wall_hug_pose.py` | 정상 frame 소비·미완성 suffix 제외·완성 오류 유지 회귀 |
| `src/go2_mujoco_lidar/test/acceptance/README.md` | 같은 overlay의 acceptance source 순서 |
| `README.md` | simulation private overlay 설치·source |
| `docs/stabilization-2026-11/README.md` | 결과 탐색 링크 |
| `docs/stabilization-2026-11/G02_RUNTIME_GATE.md` | 이번 검증·거부 전체·판정 |

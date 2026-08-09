  // 모든 /api/* 응답 401 → 런타임 정지 + 로그인 화면. (기존 fetch 호출 전부 커버)
  (function(){
    var _f = window.fetch.bind(window);
    window.fetch = function(url, opts){
      opts = Object.assign({credentials: 'same-origin'}, opts || {});
      return _f(url, opts).then(function(r){
        if (r.status === 401 && typeof url === 'string' && url.indexOf('/api/') === 0) {
          try { stopAuthenticatedRuntime(); } catch (e) {}
          show('login');
        }
        return r;
      });
    };
  })();

var GLOBALNAV=[["entry","home","전체 현황","nav"],["t-home","person","수업 보조","navT"],["s-home","shield","순찰 당직","navS"],["fleet","robot","로봇 관리","nav"],["events","list","이벤트","nav"],["t-returning","battery","홈 복귀","goReturning"]];
  var curMode=null;
  // 종합 이벤트 현황 — 교사/보안관 공통 컬럼. 클라 메모리. 시각 = YYYY-MM-DDTHH:MM:SS(로컬).
  function _p2(n){return (n<10?'0':'')+n;}
  function _todayStr(){var d=new Date();return d.getFullYear()+'-'+_p2(d.getMonth()+1)+'-'+_p2(d.getDate());}
  function _nowTs(){return _todayStr()+'T'+new Date().toTimeString().slice(0,8);}
  // 고정 시드 없음 — 초기 로그는 웹앱 로드 시 '전체 등록 로봇 현재 상태'로 기록(evBoot).
  // 서버 DB 누적 보관: 로드 시 서버 이력 조회, logEvent마다 서버 저장(새로고침·재접속·서버재시작에도 유지).
  var EVLOG=[], evLoaded=false, evBootDone=false;
  // 운영 현황 알림·이벤트 카운트 기준 시각 — 이 시점 이후 발생분만 센다(카운트만 0으로 초기화).
  // 이전 기록(EVLOG/서버 DB)은 그대로 유지, 종합 이벤트·알림 화면엔 전부 보인다.
  var OPS_T0=null;
  function opsT0(){ if(OPS_T0===null) OPS_T0=_nowTs(); return OPS_T0; }
  var EV_PAGE_SIZE=10, evPage=1, evFilterSig='', notiPage=1;
  function evGoto(p){evPage=p;renderEvents();}
  function notiGoto(p){notiPage=p;renderNoti();}
  function esc(s){return String(s==null?'':s).replace(/[&<>"']/g,function(c){return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c];});}
  function evPost(e){fetch('/api/events',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(e)}).catch(function(){});}
  function evLoadServer(){fetch('/api/events?limit=5000').then(function(x){return x.json();}).then(function(list){
    EVLOG=list||[];evLoaded=true;if(FLEET&&FLEET.length)evBoot(FLEET);renderEvents();refreshNotiBadge();}).catch(function(){});}
  // 웹앱 로드 시 1회: '웹앱 시작' + 로봇별 현재 상태(온/오프라인·저전압)를 append.
  // 중복 방지 가드: 서버 이력(EVLOG)의 직전 기록과 상태가 같으면 생략 → 새로고침·재접속마다 같은 스냅샷이 쌓이지 않음.
  function evLastByType(robot,re){for(var i=0;i<EVLOG.length;i++){var e=EVLOG[i];if(e.robot===robot&&re.test(e.type||''))return e;}return null;}
  function evBoot(list){if(evBootDone||!evLoaded)return;evBootDone=true;
    var pending=[];
    list.forEach(function(r){var ri=robotInfo(r),res=r.online?'온라인':'오프라인';
      var pr=evLastByType(ri.robot,/ 온라인$| 오프라인$/);
      if(!pr||pr.result!==res)pending.push([ri.robot+' '+res,(r.online?'가동':'미연결'),res,ri]);   // 예: 'pinky_07d7 온라인'
      if(r.online){var pb=evLastByType(ri.robot,/배터리/),lowNow=battLow(r.battery_v);
        if(lowNow&&(!pb||pb.type!=='배터리 주의'))pending.push(['배터리 주의',battStr(r.battery_v),'주의',ri]);
        else if(!lowNow&&pb&&pb.type==='배터리 주의')pending.push(['배터리 회복',battStr(r.battery_v),'정상',ri]);}});
    if(!pending.length)return;   // 변경 없음 → 부팅 스냅샷 전체 생략(중복 방지)
    logEvent('웹앱 시작','전체 등록 로봇 상태 확인','정상',{robot:'-',ip:'-',mode:'시스템',pos:'-'});
    pending.forEach(function(a){logEvent(a[0],a[1],a[2],a[3]);});}
  var evSortDesc=true;   // 시간 정렬 방향(true=최신순 내림차순 / false=오래된순 오름차순)
  var EVLABEL={
    't-following':{type:'호출 · 수업 동행 시작',status:'수업 동행 중',result:'진행'},
    't-candy':{type:'선물주기',status:'수행 중',result:'진행'},
    't-pickplace':{type:'교보재 올리기',status:'수행 중',result:'진행'},
    't-returning':{type:'홈 복귀',status:'복귀 중',result:'진행'},
    's-recycle':{type:'분리수거',status:'수행 중',result:'진행'},
    's-dispatch':{type:'현장 출동',status:'이동 중',result:'진행'},
    's-onsite':{type:'현장 확인',status:'현장 확인',result:'진행'},
    's-escalate':{type:'상황실 전파',status:'전파',result:'완료'}
  };
  function modeLabel(m){return m==='assist'?'선생님 보조':m==='patrol'?'교내 순찰':m==='idle'?'대기':(m||'-');}
  function robotInfo(r){if(!r)return {robot:'-',ip:'-',mode:'-',pos:'-'};
    return {robot:r.name||('Pinky-'+r.id),ip:r.ip||'-',mode:modeLabel(r.mode),
            pos:r.pose?('('+r.pose.x.toFixed(2)+', '+r.pose.y.toFixed(2)+')'):'-'};}
  function curRobotInfo(){return robotInfo(robotById(selectedRobotId()));}
  function logEvent(type,status,result,info){info=info||curRobotInfo();
    EVLOG.unshift({t:_nowTs(),robot:info.robot,ip:info.ip,mode:info.mode,
      type:type,result:result||'-',status:status||'-',pos:info.pos});
    if(EVLOG.length>5000)EVLOG.pop();
    evPost(EVLOG[0]);
    var s=document.getElementById('events');if(s&&s.classList.contains('active'))renderEvents();
    refreshNotiBadge();}
  function notiActiveCount(){var t0=opsT0();return EVLOG.filter(function(e){var s=(e.type||'')+' '+(e.result||'')+' '+(e.status||'');return /fire|화재|경고|긴급정지/.test(s)&&!/해소|해제/.test(s)&&(e.t||'')>t0;}).length;}   // 활성 알림(해소 제외, 세션 기준 이후만 — 뱃지 누적 0으로 초기화)
  function refreshNotiBadge(){var n=notiActiveCount(),b=document.getElementById('tb-badge');if(!b)return;b.textContent=n;b.style.display=n>0?'':'none';}   // 실제 수 반영·0이면 숨김
  // (A) 로봇 실이벤트 자동 기록 — fleet WS 델타 감시(모드 변경 / 온·오프라인 전이). 최초 관측은 baseline만.
  var EVWATCH={};
  function watchFleetEvents(list){list.forEach(function(r){var p=EVWATCH[r.id];
    var rj=r.relocalize?JSON.stringify(r.relocalize):null;   // 콘솔이 받는 relocalize_event 스냅샷
    var dj=r.dock?JSON.stringify(r.dock):null;               // 콘솔이 받는 dock_event 스냅샷
    var low=r.online?battLow(r.battery_v):(p?p.low:false);   // 저전압(<7.0V) — 오프라인이면 직전 상태 유지(오탐 방지)
    if(p){
      if(p.online&&!r.online){var oi=robotInfo(r);oi.mode=modeLabel(p.mode);logEvent(oi.robot+' 오프라인','연결 끊김','오프라인',oi);}
      else if(!p.online&&r.online){var oi2=robotInfo(r);logEvent(oi2.robot+' 온라인','연결됨','온라인',oi2);}
      if(r.online&&r.mode&&p.mode&&p.mode!==r.mode){logEvent('모드 변경: '+modeLabel(p.mode)+' → '+modeLabel(r.mode),modeLabel(r.mode),'-',robotInfo(r));}
      if(r.online){if(!p.low&&low)logEvent('배터리 주의',battStr(r.battery_v),'주의',robotInfo(r));else if(p.low&&!low)logEvent('배터리 회복',battStr(r.battery_v),'정상',robotInfo(r));}
      if(rj&&rj!==p.reloc){var rv=r.relocalize;logEvent('재측위'+(rv.tag_id!=null?' · tag'+rv.tag_id:''),(rv.event==='success'?'재측위 성공':'재측위 실패'),(rv.event==='success'?'성공':'타임아웃'),robotInfo(r));}
      if(dj&&dj!==p.dock){var dv=r.dock;logEvent('도킹'+(dv.tag_id!=null?' · tag'+dv.tag_id:''),(dv.event==='done'?'도킹 완료':dv.event==='started'?'도킹 시작':'도킹 실패'),(dv.event==='done'?'완료':dv.event==='started'?'진행':'실패'),robotInfo(r));}
    }   // 최초 관측(p 없음)은 baseline만 — 로드 시 상태 스냅샷은 evBoot가 담당(중복 방지)
    EVWATCH[r.id]={mode:r.mode,online:r.online,reloc:rj,dock:dj,low:low};});}
  function evModeTag(e){var m=(e&&e.mode)||'',s=((e&&e.type)||'')+' '+((e&&e.result)||'')+' '+((e&&e.status)||'');
    if(/해소/.test(s))return {l:'해소',c:'var(--teacher)'};   // 해소=녹색(경고보다 우선)
    if(/긴급정지/.test(s))return {l:'긴급정지',c:'var(--danger)'};   // 긴급정지·해제=빨강
    if(/트래픽제어/.test(s))return {l:'트래픽제어',c:'var(--primary)'};   // 통행 대기·해제=파랑
    if(/fire|화재|경고/.test(s))return {l:'경고',c:'#f97316'};   // 화재=경고 주황
    if(/시스템/.test(m))return {l:'시스템',c:'var(--primary)'};
    if(/실패|타임아웃|에러|오류/.test(s))return {l:'실패',c:'var(--danger)'};
    if(/오프라인|미연결|끊김/.test(s))return {l:'오프라인',c:'var(--muted)'};
    if(/온라인|연결/.test(s))return {l:'온라인',c:'var(--teacher)'};
    if(/성공|완료|정상|가동/.test(s))return {l:'정상',c:'var(--teacher)'};
    if(/보조|교사/.test(m))return {l:'수업 보조',c:'var(--teacher)'};
    if(/순찰|보안/.test(m))return {l:'순찰 당직',c:'var(--safety)'};
    if(/대기/.test(m))return {l:'대기',c:'var(--muted)'};
    return {l:(m||'-'),c:'var(--muted)'};}
  function evToggleSort(){evSortDesc=!evSortDesc;var b=document.getElementById('ev-sort');if(b)b.textContent=evSortDesc?'최신순 ↓':'오래된순 ↑';renderEvents();}
  // 이벤트 출력용 로봇 식별자 치환: 저장값(name/legacy) → 'Pinky-<id>'. 출처 = robots.yaml(/api/fleet).
  // 저장 데이터는 그대로 두고 렌더 시점에만 치환하므로 과거 행도 모두 Pinky-<id>로 표시됨.
  var EVIDMAP={};
  function evRobotLabel(s){return EVIDMAP[s]||s;}
  function evTypeLabel(t){var m=t&&t.match(/^(.+) (온라인|오프라인)$/);return (m&&EVIDMAP[m[1]])?EVIDMAP[m[1]]+' '+m[2]:t;}
  function renderEvents(){var el=document.getElementById('ev-list');if(!el)return;
    var pg=document.getElementById('ev-pager');
    var qEl=document.getElementById('ev-q'),q=qEl?qEl.value.trim().toLowerCase():'';
    var rows=EVLOG.slice().sort(function(a,b){var c=a.t<b.t?-1:(a.t>b.t?1:0);return evSortDesc?-c:c;});
    if(q)rows=rows.filter(function(e){return (e.robot+' '+evRobotLabel(e.robot)+' '+e.ip+' '+e.mode+' '+evTypeLabel(e.type)+' '+e.result+' '+e.status+' '+e.pos+' '+e.t).toLowerCase().indexOf(q)>=0;});
    var fEl=document.getElementById('ev-from'),tEl=document.getElementById('ev-to');
    if(fEl&&fEl.value)rows=rows.filter(function(e){return e.t>=(fEl.value.length===16?fEl.value+':00':fEl.value);});
    if(tEl&&tEl.value)rows=rows.filter(function(e){return e.t<=(tEl.value.length===16?tEl.value+':59':tEl.value);});
    var sig=q+'|'+(fEl?fEl.value:'')+'|'+(tEl?tEl.value:'')+'|'+evSortDesc;
    if(sig!==evFilterSig){evFilterSig=sig;evPage=1;}
    if(!rows.length){el.innerHTML='<div class="li"><div class="li-t">'+(q?'검색 결과 없음':'기록 없음')+'</div></div>';if(pg)pg.innerHTML='';return;}
    var total=rows.length,pages=Math.ceil(total/EV_PAGE_SIZE);
    if(evPage>pages)evPage=pages;if(evPage<1)evPage=1;
    var start=(evPage-1)*EV_PAGE_SIZE;
    el.innerHTML=rows.slice(start,start+EV_PAGE_SIZE).map(function(e){var tg=evModeTag(e);
      return '<div class="li"><span style="flex:0 0 auto;padding:3px 8px;border-radius:6px;font-size:11px;font-weight:700;color:#fff;background:'+tg.c+'">'+esc(tg.l)+'</span>'+
        '<div><div class="li-t">'+esc(evTypeLabel(e.type))+' · '+esc(e.result)+'</div>'+
        '<div class="li-s">'+esc(evRobotLabel(e.robot))+'('+esc(e.ip)+') · '+esc(e.mode)+' · '+esc(e.status)+' · 위치 '+esc(e.pos)+'</div></div>'+
        '<span class="li-x pill idle nodot">'+esc(e.t.slice(5,10)+' '+e.t.slice(11))+'</span></div>';}).join('');
    if(pg)pg.innerHTML=(total<=EV_PAGE_SIZE)?'<span style="font-size:12px;color:var(--muted)">전체 '+total+'건</span>':
      '<button class="btn ghost small" onclick="evGoto(evPage-1)"'+(evPage<=1?' disabled':'')+'>← 이전</button>'+
      '<span style="font-size:12.5px;color:var(--muted);min-width:130px;text-align:center">'+evPage+' / '+pages+' · 전체 '+total+'건</span>'+
      '<button class="btn ghost small" onclick="evGoto(evPage+1)"'+(evPage>=pages?' disabled':'')+'>다음 →</button>';}
  function renderNoti(){var el=document.getElementById('noti-list');if(!el)return;
    var pg=document.getElementById('noti-pager');
    var rows=EVLOG.filter(function(e){var s=(e.type||'')+' '+(e.result||'')+' '+(e.status||'');return /fire|화재|경고|긴급정지/.test(s)&&!/해소|해제/.test(s);});   // 해소는 알림 제외(활성 경보만)
    rows.sort(function(a,b){return a.t<b.t?1:(a.t>b.t?-1:0);});
    if(!rows.length){el.innerHTML='<div class="li"><div class="li-t">알림 없음</div></div>';if(pg)pg.innerHTML='';return;}
    var total=rows.length,pages=Math.ceil(total/EV_PAGE_SIZE);
    if(notiPage>pages)notiPage=pages;if(notiPage<1)notiPage=1;
    var start=(notiPage-1)*EV_PAGE_SIZE;
    el.innerHTML=rows.slice(start,start+EV_PAGE_SIZE).map(function(e){
      return '<div class="li"><span class="ic" data-ic="alert" style="color:#f97316"></span>'+
        '<div><div class="li-t">화재 감지 — '+esc(e.pos||e.result||'')+'</div>'+
        '<div class="li-s">'+esc(e.robot||'천장CCTV')+' · '+esc(e.t.slice(5,10)+' '+e.t.slice(11))+'</div></div>'+
        '<span class="li-x pill alert nodot">경고</span></div>';}).join('');
    paintIcons(el);
    if(pg)pg.innerHTML=(total<=EV_PAGE_SIZE)?'<span style="font-size:12px;color:var(--muted)">전체 '+total+'건</span>':
      '<button class="btn ghost small" onclick="notiGoto(notiPage-1)"'+(notiPage<=1?' disabled':'')+'>← 이전</button>'+
      '<span style="font-size:12.5px;color:var(--muted);min-width:130px;text-align:center">'+notiPage+' / '+pages+' · 전체 '+total+'건</span>'+
      '<button class="btn ghost small" onclick="notiGoto(notiPage+1)"'+(notiPage>=pages?' disabled':'')+'>다음 →</button>';}
  function renderPatrolFireCard(){var el=document.getElementById('s-eventfeed');if(!el)return;
    var fire=null;
    for(var i=0;i<EVLOG.length;i++){if(EVLOG[i].robot==='천장CCTV'){fire=EVLOG[i];break;}}   // 최신 천장CCTV 화재
    if(!fire||/해소/.test(fire.status||'')||/해소/.test(fire.type||'')){eventOn=false;el.innerHTML='';return;}   // 없거나 해소면 카드 숨김
    var z=(String(fire.pos||fire.result||'').match(/Zone([A-E])/)||[0,'E'])[1];
    curEvent='fire';eventOn=true;EVENTS.fire.zone=z;   // 라이브 천장CCTV 이벤트 = 대응 화면 활성
    el.innerHTML='<div class="event" onclick="openEvent(\'fire\')"><span class="ic" data-ic="alert"></span>'+
      '<div><div class="et">'+ETITLE.fire+'</div><div class="es">'+zlabel(z)+' · 천장 CCTV 1차 감지</div></div>'+
      '<span class="sev">출동 필요 <span class="ic" data-ic="back" style="transform:rotate(180deg)"></span></span></div>';
    paintIcons(el);refreshNotiBadge();}
  var armTestMode='left',armTestTimer=null,armTestLogId=0;
  function armTestHeaders(){return {'Content-Type':'application/json'};}
  async function armTestJson(url,opts){
    var response=await fetch(url,opts);var data=await response.json();
    if(!response.ok)throw new Error(data.detail||'로봇팔 요청 실패');return data;
  }
  function setArmTestMode(mode){
    armTestMode=mode;
    document.querySelectorAll('[data-arm-test-mode]').forEach(function(b){
      var on=b.dataset.armTestMode===mode;b.classList.toggle('active',on);b.classList.toggle('ghost',!on);
    });
    document.querySelectorAll('[data-arm-test-modes]').forEach(function(el){
      el.style.display=el.dataset.armTestModes.split(' ').indexOf(mode)>=0?'':'none';
    });
    document.getElementById('arm-test-camera-panel').style.display=mode==='dual'?'none':'';
    armTestLogId=0;refreshArmTest();
  }
  var armFeatureNames={'face-recognition':'얼굴인식','fire-detect':'화재감지','tracking':'추종'};
  function paintArmFeatureToggles(status){
    document.querySelectorAll('[data-arm-feature-toggle]').forEach(function(button){
      var feature=button.dataset.armFeatureToggle;
      var running=!!(status[feature]&&status[feature].running);
      button.classList.toggle('arm-toggle-on',running);
      button.setAttribute('aria-pressed',running?'true':'false');
      button.textContent=armFeatureNames[feature]+' · '+(running?'ON':'OFF');
    });
  }
  async function refreshArmTest(){
    if(!document.getElementById('arm-test').classList.contains('active'))return;
    var server=document.getElementById('arm-test-server'),feature=document.getElementById('arm-test-feature');
    try{
      var status=await armTestJson('/api/arm/status?arm_id='+armTestMode);
      server.textContent='서버 연결됨';server.className='pill ok';
      if(armTestMode==='dual')feature.textContent='듀얼암 · '+(status.phase||status.status||'대기');
      else{
        var active=Object.keys(status).filter(function(k){return status[k]&&status[k].running;});
        feature.textContent=active.length?('실행 중 · '+active.join(', ')):'비전 기능 OFF';
        paintArmFeatureToggles(status);
      }
      if(armTestMode!=='dual')document.getElementById('arm-test-camera').src='/api/arm/camera?arm_id='+armTestMode+'&t='+Date.now();
    }catch(err){server.textContent=err.message;server.className='pill alert';feature.textContent='상태 확인 실패';}
    try{
      var firstLoad=armTestLogId===0;
      var data=await armTestJson('/api/arm/logs?after_id='+armTestLogId),logs=data.logs||[];
      if(logs.length){
        armTestLogId=logs[logs.length-1].id;
        var el=document.getElementById('arm-test-logs');
        if(firstLoad)el.textContent='';
        logs.forEach(function(x){el.textContent+='['+(x.timestamp_iso||'')+'] ['+(x.source||'-')+'] '+x.message+'\n';});
        el.scrollTop=el.scrollHeight;
      }
    }catch(err){}
  }
  async function armTestCommand(command){
    if(!confirm(armTestMode.toUpperCase()+' · '+command+' 명령을 실행할까요?'))return;
    try{await armTestJson('/api/arm/command',{method:'POST',headers:armTestHeaders(),body:JSON.stringify({arm_id:armTestMode,command:command})});await refreshArmTest();}
    catch(err){alert(err.message);}
  }
  var mappedArmTasks={
    gift:{label:'선물주기',arm_id:'dual',command:'gift-giving',robot:'Dual Arm'},
    teaching:{label:'교보재 올리기',arm_id:'left',command:'help',robot:'Left Arm'},
    recycle:{label:'분리수거',arm_id:'left',command:'recycle',robot:'Left Arm'}
  };
  async function runMappedArmTask(taskKey,button){
    var task=mappedArmTasks[taskKey];
    if(!task||!confirm(task.label+' 기능을 실행할까요?'))return;
    var status=button&&button.dataset.statusId?document.getElementById(button.dataset.statusId):null;
    var oldText=button&&!status?button.innerHTML:'';
    if(button)button.disabled=true;
    if(status){status.textContent='명령 전송 중…';status.className='sub home-arm-status running';}
    else if(button)button.textContent='명령 전송 중…';
    try{
      await armTestJson('/api/arm/command',{method:'POST',headers:armTestHeaders(),
        body:JSON.stringify({arm_id:task.arm_id,command:task.command})});
      logEvent(task.label,task.robot+' 명령 전송','진행',
        {robot:task.robot,ip:'-',mode:'로봇팔',pos:'-'});
      if(status){
        status.textContent='실행 요청 완료';status.className='sub home-arm-status done';
        button.disabled=false;
      }else if(button){
        button.textContent='명령 전송 완료';
        setTimeout(function(){button.disabled=false;button.innerHTML=oldText;paintIcons(button);},1500);
      }
    }catch(err){
      if(status){status.textContent='실행 실패';status.className='sub home-arm-status failed';button.disabled=false;}
      else if(button){button.disabled=false;button.innerHTML=oldText;paintIcons(button);}
      alert(task.label+' 실행 실패: '+err.message);
    }
  }
  async function armTestFeature(feature){
    try{
      var button=document.querySelector('[data-arm-feature-toggle="'+feature+'"]');
      if(button)button.disabled=true;
      await armTestJson('/api/arm/feature',{method:'POST',headers:armTestHeaders(),body:JSON.stringify({arm_id:armTestMode,feature:feature})});
      await refreshArmTest();
    }
    catch(err){alert(err.message);}
    finally{var button=document.querySelector('[data-arm-feature-toggle="'+feature+'"]');if(button)button.disabled=false;}
  }
  function startArmTest(){setArmTestMode(armTestMode);if(!armTestTimer)armTestTimer=setInterval(refreshArmTest,1500);}
  function stopArmTest(){if(armTestTimer){clearInterval(armTestTimer);armTestTimer=null;}var img=document.getElementById('arm-test-camera');if(img)img.removeAttribute('src');}

  // Admin GUI와 동일한 로봇팔 알림을 통합 GUI에서도 전역 감시한다.
  var armPromptTimer=null;
  var armSeenFire={left:null,right:null},armSeenFace={left:null,right:null};
  async function pollArmPromptFor(armId){
    try{
      var fire=await armTestJson('/api/arm/fire-prompt?arm_id='+armId+'&t='+Date.now());
      var fp=fire.prompt;
      if(fp&&fp.id!==armSeenFire[armId]){
        armSeenFire[armId]=fp.id;
        var yes=confirm((fp.title||'화재 감지')+' · '+armId.toUpperCase()+' ARM\n\n'+fp.message+
          '\n\n확인: '+(fp.yes_label||'예')+' / 취소: '+(fp.no_label||'아니오'));
        try{await armTestJson('/api/arm/fire-response?arm_id='+armId,{method:'POST',headers:armTestHeaders(),body:JSON.stringify({response:yes?'yes':'no'})});}
        catch(err){alert('화재 대응 전송 실패: '+err.message);}
      }
      var face=await armTestJson('/api/arm/face-prompt?arm_id='+armId+'&t='+Date.now());
      var xp=face.prompt;
      if(xp&&xp.id!==armSeenFace[armId]){
        armSeenFace[armId]=xp.id;
        alert((xp.title||'얼굴인식 결과')+' · '+armId.toUpperCase()+' ARM\n\n'+xp.message);
        try{await armTestJson('/api/arm/face-prompt/ack?arm_id='+armId,{method:'POST',headers:armTestHeaders()});}
        catch(err){alert('얼굴인식 알림 확인 실패: '+err.message);}
      }
    }catch(err){/* 로봇팔 서버 미연결은 로봇팔 테스트 상태창에서 표시 */}
  }
  function pollArmPrompts(){if(!runtimeActive)return;pollArmPromptFor('left');pollArmPromptFor('right');}
  function startArmPrompts(){if(!armPromptTimer){pollArmPrompts();armPromptTimer=setInterval(pollArmPrompts,1500);}}
  function stopArmPrompts(){if(armPromptTimer){clearInterval(armPromptTimer);armPromptTimer=null;}}

  function show(id){document.querySelectorAll('.screen').forEach(function(s){s.classList.remove('active')});var el=document.getElementById(id);if(el)el.classList.add('active');var _app=document.querySelector('.app');if(_app)_app.classList.toggle('at-login',id==='login');window.scrollTo(0,0);renderTabs(id);if(id==='t-returning'){startDock();startDockCam()}else{stopDock();stopDockCam()}closeMenu();if(id==='events')renderEvents();if(id==='noti')renderNoti();if(id==='s-home'){renderPatrolFireCard();refreshPatrolUI();paintPatrolRobots(FLEET)}if(id==='t-home'){paintPatrolRobots(FLEET);refreshFaceGate()}else stopMotion();if(id==='entry')paintHome(FLEET);if(id==='t-following')applyAssistUI();if(id==='fleet'){startCctv();startFleetCam()}else{stopCctv();stopFleetCam()}if(id==='arm-test')startArmTest();else stopArmTest();if(/^s-(event|dispatch|onsite|escalate)$/.test(id))applyEventUI(id);
    // 세션 없이 메뉴로 열어본 t-following은 '동행 시작' 이벤트를 남기지 않는다.
    if(EVLABEL[id]&&!(id==='t-following'&&!assistOn))logEvent(EVLABEL[id].type,EVLABEL[id].status,EVLABEL[id].result)}
  // P6 카메라(B): 홈복귀 진입 시 선택 로봇 도킹 전방영상 WS(binary JPEG) → dockcam-img.
  var dockCamWs=null,dockCamUrl=null;
  function startDockCam(){
    stopDockCam();
    var id=selectedRobotId(),img=document.getElementById('dockcam-img');
    if(id===null||!img)return;
    try{
      dockCamWs=new WebSocket((location.protocol==='https:'?'wss':'ws')+'://'+location.host+'/ws/camera?robot='+id);
      dockCamWs.binaryType='blob';
      dockCamWs.onmessage=function(e){
        var url=URL.createObjectURL(e.data);img.src=url;img.style.display='block';
        if(dockCamUrl)URL.revokeObjectURL(dockCamUrl);dockCamUrl=url;
      };
    }catch(err){}
  }
  function stopDockCam(){
    if(dockCamWs){try{dockCamWs.close();}catch(x){}dockCamWs=null;}
    if(dockCamUrl){URL.revokeObjectURL(dockCamUrl);dockCamUrl=null;}
    var img=document.getElementById('dockcam-img');if(img){img.removeAttribute('src');img.style.display='none';}
  }
  // 천장 CCTV(topview): 로봇 관리 화면에서 /ws/cctv(binary JPEG) → cctv-img. 전방 카메라와 동일 패턴.
  var cctvWs=null,cctvUrl=null;
  // 로봇 관리 화면 전방 카메라 — 도킹/재측위로 detector 가 뜨면 프레임이 들어온다.
  // 프레임이 오는 동안만 천장 CCTV 자리를 전방 뷰로 바꾸고, 3초 끊기면 천장으로 되돌린다.
  var fcamWs=null, fcamUrl=null, fcamLast=0, fcamTimer=null;
  function fcamShow(on){
    var f=document.getElementById('fleetcam-img'),c=document.getElementById('cctv-img'),
        n=document.getElementById('cctv-note'),l=document.getElementById('ccam-label');
    if(!f||!c)return;
    f.style.display=on?'block':'none';
    c.style.display=on?'none':(c.getAttribute('src')?'block':'none');
    if(l)l.textContent=on?'TOP / 전방 View · 전방(도킹 중)':'TOP / 전방 View';
    if(n&&on)n.style.display='none';
  }
  function startFleetCam(){
    stopFleetCam();
    var id=selectedRobotId(),f=document.getElementById('fleetcam-img');
    if(id===null||!f)return;
    fcamWs=new WebSocket((location.protocol==='https:'?'wss':'ws')+'://'+location.host
                         +'/ws/camera?robot='+id);
    fcamWs.binaryType='blob';
    fcamWs.onmessage=function(e){
      var url=URL.createObjectURL(e.data);
      if(fcamUrl)URL.revokeObjectURL(fcamUrl);fcamUrl=url;
      f.src=url;fcamLast=Date.now();fcamShow(true);
    };
    fcamTimer=setInterval(function(){                 // 3초 무프레임 → 천장으로 복귀
      if(fcamLast&&Date.now()-fcamLast>3000){fcamLast=0;fcamShow(false)}
    },1000);
  }
  function stopFleetCam(){
    if(fcamWs){try{fcamWs.close()}catch(x){}fcamWs=null}
    if(fcamTimer){clearInterval(fcamTimer);fcamTimer=null}
    if(fcamUrl){URL.revokeObjectURL(fcamUrl);fcamUrl=null}
    var f=document.getElementById('fleetcam-img');
    if(f)f.removeAttribute('src');
    fcamLast=0;fcamShow(false);
  }

  function startCctv(){
    stopCctv();
    var img=document.getElementById('cctv-img'),note=document.getElementById('cctv-note');
    if(!img)return;
    cctvWs=new WebSocket((location.protocol==='https:'?'wss':'ws')+'://'+location.host+'/ws/cctv');
    cctvWs.binaryType='blob';
    cctvWs.onmessage=function(e){
      var url=URL.createObjectURL(e.data);
      if(cctvUrl)URL.revokeObjectURL(cctvUrl);cctvUrl=url;
      img.src=url;if(!fcamLast)img.style.display='block';   // 전방 표시 중이면 천장은 숨김 유지
      if(note)note.style.display='none';
    };
  }
  function stopCctv(){
    if(cctvWs){try{cctvWs.close();}catch(x){}cctvWs=null;}
    if(cctvUrl){URL.revokeObjectURL(cctvUrl);cctvUrl=null;}
    var img=document.getElementById('cctv-img'),note=document.getElementById('cctv-note');
    if(img){img.removeAttribute('src');img.style.display='none';}if(note)note.style.display='';
  }
  var dockTimer=null,dockT=0;
  function startDock(){updateReturning()}   // 실 dock_event/tag 기반(가짜 애니메이션 제거)
  function stopDock(){if(dockTimer){clearInterval(dockTimer);dockTimer=null}}
  // 타임라인 li 상태 갱신(class + .tl-s 텍스트)
  function tlSet(liId,cls,sub){var li=document.getElementById(liId);if(!li)return;li.className=cls;var s=li.querySelector('.tl-s');if(s)s.textContent=sub}
  // 홈 복귀 화면을 현재 선택 로봇의 실 상태(tag/relocalize/dock)로 갱신
  function updateReturning(){
    var id=selectedRobotId(),r=(id!==null)?robotById(id):null;
    var pe=document.getElementById('dockpose');
    if(pe){var t=r&&r.tag;
      pe.textContent=(t&&t.tag_id===HOME_TAG)?('tag'+t.tag_id+'  거리 '+t.dist_cm+'cm  측면 '+t.lateral_cm+'cm'):'태그 탐색 중…';}
    // 현재 홈복귀 대상(HOME_TAG)과 일치하는 이벤트만 표시 — 다른 태그의 옛(stale) 성공/실패가 뜨지 않게
    var rl=r&&r.relocalize, dk=r&&r.dock;
    var relocOk=rl&&rl.event==='success'&&rl.tag_id===HOME_TAG;
    var dkCur=(dk&&dk.tag_id===HOME_TAG)?dk:null;
    if(relocOk)tlSet('tl-reloc','done','완료 (tag'+rl.tag_id+')');
    else if(dkCur&&dkCur.event==='done')tlSet('tl-reloc','done','완료');
    else tlSet('tl-reloc','active','보정 중…');
    if(dkCur&&dkCur.event==='done')tlSet('tl-dock','done','완료');
    else if(dkCur&&dkCur.event==='failed')tlSet('tl-dock','active','실패'+(dkCur.reason?' ('+dkCur.reason+')':''));
    else if(dkCur&&dkCur.event==='started')tlSet('tl-dock','active','정밀 정차 중…');
    else tlSet('tl-dock','pending','대기');
    // 도킹 완료 시 제목을 자동으로 '복귀 완료'로
    var ttl=document.getElementById('returning-title');
    if(ttl)ttl.textContent=(dkCur&&dkCur.event==='done')?'복귀 완료':'홈으로 복귀 중';
  }
  function renderTabs(id){var bar=document.getElementById('tabbar');if(id==='login'){bar.style.display='none';return}bar.style.display='flex';
    var ids=GLOBALNAV.map(function(t){return t[0]});
    var act=(ids.indexOf(id)>=0)?id:(id.indexOf('t-')===0?'t-home':(id.indexOf('s-')===0?'s-home':''));
    bar.innerHTML=GLOBALNAV.map(function(t){var on=(t[0]===act)?'on':'';
      var badge=(t[0]==='events'&&notiActiveCount()>0)?'<span class="badge" style="top:2px;right:calc(50% - 18px)">'+notiActiveCount()+'</span>':'';
      return '<button class="tab '+on+'" onclick="'+t[3]+'(\''+t[0]+'\')">'+badge+'<span class="ic" data-ic="'+t[1]+'"></span>'+t[2]+'</button>';}).join('');
    paintIcons(bar);}
  function setMode(m){curMode=m;var mode=document.getElementById('tb-mode');if(m==='teacher'){mode.textContent='수업 보조';mode.className='modepill'}else{mode.textContent='순찰 당직';mode.className='modepill safety'}}
  function enterTeacher(){setMode('teacher');show('t-home');logEvent('수업 보조 진입','보조 대기','완료')}   // 진입은 대기 유지 — 보조는 '호출'부터
  function enterSafety(){setMode('safety');show('s-home');logEvent('순찰 당직 진입','순찰 대기','완료')}   // 진입은 대기 유지 — 순찰은 '순찰 시작'부터
  function nav(id){curMode=null;show(id)}
  // 홈 복귀(도킹) 게이트 — 실제 도킹 명령이 나가므로 로봇 확인 + 사용자 확인 후에만.
  // 세 진입점(하단 '충전'·드로어 '홈 복귀'·수업 동행 '홈 복귀')이 모두 이 함수를 거친다.
  function goReturning(){
    var id=selectedRobotId();
    if(id===null){alert('먼저 로봇을 선택하세요.');return;}
    if(!confirm('Pinky-'+id+'을(를) 홈(tag8)으로 복귀시킬까요?'))return;
    sendDock();show('t-returning');
  }
  function navT(id){setMode('teacher');show(id)}
  function navS(id){setMode('safety');show(id)}
  function finishReturn(){assistOn=false;document.getElementById('tb-mode').textContent='대기';document.getElementById('tb-mode').className='modepill idle';show('t-home');sendMode('idle')}
  function stopToIdle(){assistOn=false;document.getElementById('tb-mode').textContent='대기';document.getElementById('tb-mode').className='modepill idle';show('t-home');sendMode('idle')}

  // 수업 동행 세션 — '호출'로 시작했을 때만 true. 메뉴로 화면만 열어본 경우와 구분한다.
  var assistOn=false;
  function startAssist(){assistOn=true;sendMode('assist');show('t-following')}
  function stopAssist(){stopToIdle()}   // 동행 정지 = 대기로 복귀 + cmd_mode idle 발행(assistOn도 해제)
  // t-following 화면을 실제 세션 상태에 맞춰 표시(세션 없으면 대기 문구 + 태스크 비활성).
  function applyAssistUI(){
    var t=document.getElementById('tfol-title'),p=document.getElementById('tfol-pill'),
        d=document.getElementById('tfol-dist'),sd=document.getElementById('st-dist-tfol'),
        box=document.getElementById('tfol-tasks');
    if(!t||!p||!box)return;
    t.textContent=assistOn?'수업 동행 중':'수업 동행 대기';
    // 상태 문구 = 로그인 계정 + 실제 도킹 진행상태(dock_event). 고정 문구 쓰지 않는다.
    var who=(window.__account||'')+' 선생님';
    var r=robotById(selectedRobotId()), dv=r&&r.dock, st='', cls='pill ok';
    if(dv&&dv.event==='started'){st=' : 교실 이동 중';cls='pill run'}
    else if(dv&&dv.event==='done'){st=' : 교실 도착 완료'}
    else if(dv&&dv.event){st=' : 이동 실패';cls='pill alert'}
    p.textContent=assistOn?(who+' 수업 동행 중'+st):'호출하면 시작됩니다';
    p.className=assistOn?cls:'pill idle';
    if(d)d.style.display='none';                 // 거리(0.7m)는 추종 미구현이라 가짜값 — 표시 안 함
    if(sd&&sd.parentElement)sd.parentElement.style.display='none';
    box.querySelectorAll('button').forEach(function(b){b.disabled=!assistOn});
  }
  // 현재 선택 로봇(상단칩 #tb-robot) id → cmd_mode 실 발행. 미선택/파싱불가면 무발행.
  function selectedRobotId(){var el=document.getElementById('tb-robot');var m=(el?el.textContent:'').match(/(\d+)/);return m?parseInt(m[1],10):null}
  function sendMode(mode){var id=selectedRobotId();if(id===null)return;
    fetch('/api/cmd/mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({robot_id:id,mode:mode})}).then(function(x){if(!x.ok)throw new Error('cmd_mode '+x.status)}).catch(function(e){alert('모드 전환 발행 실패: '+e.message)})}
  var HOME_TAG=8;   // 홈 도크 AprilTag(추후 변경 가능). 홈 복귀 = 이 태그로 도킹.
  // 홈 복귀: 현재 선택 로봇을 홈 태그로 도킹(POST /api/cmd/dock). 재측위·PID는 로봇측 자동.
  // 지정 태그로 도킹(교실이동 등). 기존 sendDock 은 홈 태그 고정이라 분리.
  function sendDockTag(tag){var id=selectedRobotId();
    if(id===null){alert('먼저 로봇을 선택하세요.');return}   // 조용히 무시하면 원인을 못 찾는다
    fetch('/api/cmd/dock',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({robot_id:id,tag_id:tag})})
      .then(function(x){if(!x.ok)throw new Error('dock '+x.status)})
      .catch(function(e){alert('이동 명령 발행 실패: '+e.message)})}
  function sendDock(){var id=selectedRobotId();if(id===null)return;
    fetch('/api/cmd/dock',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({robot_id:id,tag_id:HOME_TAG})}).then(function(x){if(!x.ok)throw new Error('dock '+x.status)}).catch(function(e){alert('도킹 발행 실패: '+e.message)})}
  // 재측위 — 콘솔과 동일 경로(POST /api/cmd/relocalize → 로봇 relocalize_cmd:detector:start). 로봇별 버튼에서 호출.
  function sendRelocalize(id){
    if(!confirm('Pinky-'+id+' 재측위를 시작할까요?'))return;
    fetch('/api/cmd/relocalize',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({robot_id:id,action:'start'})})
      .then(function(x){if(!x.ok)throw new Error('relocalize '+x.status)}).catch(function(e){alert('재측위 발행 실패: '+e.message)})}
  // 개별 정지 — 콘솔과 동일 계약(cmd_mode idle). 해당 로봇만 임무 중지 → 대기 복귀. estop(전체)과 무관.
  function sendStop(id){
    if(!confirm('Pinky-'+id+' 즉시 정지할까요? (긴급정지)'))return;
    logEvent('긴급정지 · Pinky-'+id,'개별 즉시 정지','정지',{robot:'Pinky-'+id,ip:'-',mode:'긴급정지',pos:'-'});
    estopFetch(id,true).catch(function(e){alert('정지 발행 실패: '+e.message)});}
  async function sendArmStop(armId){
    var label=armId==='dual'?'Dual Arm':(armId==='left'?'Left Arm':'Right Arm');
    if(!confirm(label+'을 즉시 정지할까요?'))return;
    try{
      await armTestJson('/api/arm/command',{method:'POST',headers:armTestHeaders(),
        body:JSON.stringify({arm_id:armId,command:'stop'})});
      logEvent('긴급정지 · '+label,'개별 로봇팔 정지','정지',
        {robot:label,ip:'-',mode:'긴급정지',pos:'-'});
      closeEstop();
    }catch(err){alert(label+' 정지 실패: '+err.message);}
  }
  // 에이전트 토글 — 온라인이면 끄기(stop), 오프라인이면 켜기(start). 백엔드가 로봇에 SSH로 스크립트 실행.
  function toggleAgent(id,online){
    var act=online?'stop':'start';
    if(!confirm('Pinky-'+id+' 에이전트를 '+(online?'끌까요? (heartbeat 끊김)':'켤까요?')))return;
    fetch('/api/cmd/agent',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({robot_id:id,action:act})})
      .then(function(x){if(!x.ok)return x.text().then(function(t){throw new Error(x.status+' '+t)});})
      .then(function(){/* 반영은 heartbeat 갱신으로 자동 */})
      .catch(function(e){alert('에이전트 '+(online?'끄기':'켜기')+' 실패: '+e.message)})}
  var BATT={'Pinky-31':'80%','Pinky-50':'74%','Pinky-87':'30%'};
  var userPicked=false;   // 사용자가 로봇을 직접 선택했는지 — 이후 자동선택이 덮어쓰지 않게
  function selectRobot(name,label,kind){
    userPicked=true;
    document.getElementById('tb-robot').textContent=name;
    var m=document.getElementById('tb-mode');m.textContent=label;
    m.className='modepill'+(kind==='safety'?' safety':kind==='idle'?' idle':'');
    nav('entry');   // 로봇 선택 → 항상 모드 선택 페이지(전체 홈)로. 교사/보안관 모드는 여기서 고른다.
    refreshBatteries();   // 상단칩+화면 배터리를 이 로봇 실값으로
  }

  var patrolling=false;
  function setPatrolUI(running){patrolling=running;document.getElementById('s-patroltext').innerHTML=running?'순찰 정지<span class="sub">순회 중지하고 대기</span>':'순찰 시작<span class="sub">지정 경로 자율 순회</span>';document.getElementById('s-patrolbtn').className='btn '+(running?'ghost':'safety');paintIcons(document.getElementById('s-patrolbtn'));document.getElementById('s-patrolcard').style.display=running?'block':'none'}
  // 서버 권위 상태로 버튼 동기(선택 로봇 기준). s-home 진입·로봇전환·새로고침 시 호출.
  function refreshPatrolUI(){var id=selectedRobotId();if(id===null){setPatrolUI(false);return}fetch('/api/patrol/status?robot_id='+id).then(function(x){return x.ok?x.json():{running:false}}).then(function(j){setPatrolUI(!!j.running)}).catch(function(){})}
  function patrolCmd(action,id){return fetch('/api/patrol/'+action,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({robot_id:id})}).then(function(x){if(!x.ok)throw new Error('patrol '+action+' '+x.status);return true})}
  function togglePatrol(){var id=selectedRobotId();
    if(id===null){alert('로봇을 먼저 선택하세요');return}
    if(!patrolling){                                   // 시작: 확인 → start API 성공 후에만 UI 갱신
      if(!confirm('로봇'+id+' 순찰을 시작할까요? 경로를 자율 주행합니다'))return;
      patrolCmd('start',id).then(function(){sendMode('patrol');setPatrolUI(true)}).catch(function(e){alert('순찰 시작 실패: '+e.message)});
    }else{                                             // 정지: stop API 성공 후에만 UI 갱신
      patrolCmd('stop',id).then(function(){sendMode('idle');setPatrolUI(false)}).catch(function(e){alert('순찰 정지 실패: '+e.message)});
    }}
  // 전체 맵을 5등분한 감시 구역(ZoneA~E). 각 Zone에 사전 셋팅된 nav2_approach 좌표로 출동. 경계 이벤트는 가까운 Zone으로 귀속.
  var ZONES={
    A:{name:'충전소',     pos:'남서', ap:{x:-3.00,y:-1.50,yaw:45}},
    B:{name:'교실',       pos:'남동', ap:{x:3.00,y:-1.50,yaw:135}},
    C:{name:'분리수거장', pos:'북서', ap:{x:-3.00,y:1.50,yaw:-45}},
    D:{name:'정원',       pos:'북동', ap:{x:3.00,y:1.50,yaw:-135}},
    E:{name:'교무실',     pos:'중앙', ap:{x:0.00,y:0.00,yaw:0}}
  };
  function zlabel(z){return 'Zone'+z+' · '+ZONES[z].name}
  // 로봇 실 위치(map 좌표) → 감시 구역. 맵 중앙 밴드=E(교무실), 나머지는 4분면(NW=C, NE=D, SW=A, SE=B).
  // 기준점은 map.json(현 wasab_map11)에서 계산 — 맵이 바뀌어도 따라간다. +y = 북(위치맵 마커와 동일 규약).
  function zoneOf(x,y){
    if(!MAPMETA||typeof x!=='number'||typeof y!=='number')return null;
    var W=MAPMETA.width*MAPMETA.resolution, H=MAPMETA.height*MAPMETA.resolution,
        cx=MAPMETA.origin[0]+W/2, cy=MAPMETA.origin[1]+H/2;
    if(Math.abs(x-cx)<W*0.18&&Math.abs(y-cy)<H*0.18)return 'E';
    return (y>=cy)?((x>=cx)?'D':'C'):((x>=cx)?'B':'A');
  }
  function zap(z){var a=ZONES[z].ap;return 'x='+a.x.toFixed(2)+'m  y='+a.y.toFixed(2)+'m  yaw='+a.yaw+'°'}
  function paintZoneLegend(){var el=document.getElementById('zone-legend');if(!el)return;var h='<span style="font-size:12px;color:var(--muted);margin-right:2px">감시 구역</span>';['A','B','C','D','E'].forEach(function(z){h+='<span class="pill idle nodot">Zone'+z+' '+ZONES[z].name+'</span>'});el.innerHTML=h}
  var EVENTS={
    fire:     {zone:'E', cam:'CCTV-3', sev:'심각도 높음', det:'fire',      conf:0.88, note:'열·연기 패턴을',   dist:12, org:'119 소방', reason:'화재'},
    intruder: {zone:'D', cam:'CCTV-5', sev:'심각도 높음', det:'intruder',  conf:0.83, note:'미등록 인물을',   dist:18, org:'112 경찰', reason:'외부인 침입'},
    violence: {zone:'B', cam:'CCTV-2', sev:'심각도 높음', det:'violence',  conf:0.79, note:'몸싸움 동작 패턴을', dist:9,  org:'112 경찰', reason:'폭력 상황'},
    security: {zone:'C', cam:'CCTV-7', sev:'심각도 높음', det:'intrusion', conf:0.81, note:'보안구역 침입을',  dist:22, org:'112 경찰', reason:'보안구역 침입'}
  };
  var ETITLE={fire:'화재 의심 감지',intruder:'외부인 감지',violence:'폭력 상황 감지',security:'보안구역 침입 감지'};
  var curEvent='fire';
  function raiseEvent(type){curEvent=type;eventOn=true;logEvent('이벤트 발생: '+ETITLE[type],'감지','미처리');var e=EVENTS[type];document.getElementById('s-eventfeed').innerHTML='<div class="event" onclick="openEvent(\''+type+'\')"><span class="ic" data-ic="alert"></span><div><div class="et">'+ETITLE[type]+'</div><div class="es">'+zlabel(e.zone)+' · 천장 CCTV 1차 감지 · 방금</div></div><span class="sev">출동 필요 <span class="ic" data-ic="back" style="transform:rotate(180deg)"></span></span></div>';paintIcons(document.getElementById('s-eventfeed'));show('s-home')}
  function openEvent(type){curEvent=type;eventOn=true;var e=EVENTS[type];document.getElementById('ev-title').textContent=ETITLE[type];document.getElementById('ev-meta').innerHTML='<span class="pill alert">미처리</span><span class="pill idle nodot">'+zlabel(e.zone)+' · '+e.cam+'</span><span class="pill idle nodot">'+e.sev+'</span>';document.getElementById('ev-bboxlbl').textContent=e.det+' '+e.conf.toFixed(2);document.getElementById('ev-note').textContent='천장 카메라가 '+e.note+' 1차 감지한 장면입니다. 로봇을 보내 현장에서 다시 확인하세요.';document.getElementById('ev-dispatchsub').textContent=zlabel(e.zone)+' 사전 nav2_approach 좌표로 이동 후 2차 확인';document.getElementById('esc-org').textContent=e.org;document.getElementById('esc-reason').textContent=e.reason+' · '+zlabel(e.zone);show('s-event')}
  function goDispatch(){var e=EVENTS[curEvent];document.getElementById('disp-zone').textContent=zlabel(e.zone);document.getElementById('disp-zone2').textContent=zlabel(e.zone);document.getElementById('disp-coord').textContent=zap(e.zone);document.getElementById('disp-dist').textContent='약 '+e.dist+'m 남음…';show('s-dispatch')}
  function goOnsite(){var e=EVENTS[curEvent];document.getElementById('onsite-lbl').textContent=e.det+' '+e.conf.toFixed(2)+' · 2차확인';show('s-onsite')}
  function resolveEvent(){var z=(EVENTS[curEvent]&&EVENTS[curEvent].zone)||'';   // 해소를 서버에 기록(천장CCTV) → 카드 자동소멸·종합이벤트 반영
    logEvent((ETITLE[curEvent]||'이벤트')+' 해소','해소','해소',{robot:'천장CCTV',ip:'',mode:'',pos:z?('Zone'+z):''});
    eventOn=false;document.getElementById('s-eventfeed').innerHTML='';show('s-history')}

  // 이벤트 대응 화면(감지 상황·출동·현장·전파) — 감지된 상황이 없으면 예시 화면임을 알리고 조치 버튼을 잠근다.
  var eventOn=false;
  function applyEventUI(id){var sec=document.getElementById(id);if(!sec)return;
    var note=sec.querySelector('.idlenote');if(note)note.style.display=eventOn?'none':'block';
    sec.querySelectorAll('button').forEach(function(b){
      var h=b.getAttribute('onclick')||'';
      b.disabled=!eventOn&&h.indexOf('show(')!==0;});   // 화면 이동 버튼은 살려 둔다
  }

  function openMenu(){document.getElementById('drawer').classList.add('open');document.getElementById('scrim').classList.add('open')}
  function closeMenu(){document.getElementById('drawer').classList.remove('open');document.getElementById('scrim').classList.remove('open')}
  var estopPick=null;   // 개별 정지 확인용 선택 장치(이동로봇/로봇팔).
  function openEstop(){estopPick=null;paintEstopRobots();document.getElementById('estop-scrim').classList.add('open');closeMenu()}
  function closeEstop(){document.getElementById('estop-scrim').classList.remove('open')}
  var ROBOTS=[
    {id:'Pinky-31',rid:31,raw:'idle',online:false,mode:'교사 보조 · 수업 동행 중',kind:'teacher'},
    {id:'Pinky-50',rid:50,raw:'idle',online:false,mode:'순찰중',kind:'safety'},
    {id:'Pinky-87',rid:87,raw:'idle',online:false,mode:'대기 · 충전소',kind:'idle'}
  ];
  function kindColor(k){return k==='teacher'?'var(--teacher)':k==='safety'?'var(--safety)':'var(--muted)'}
  // 이동로봇은 온라인 장치만, 로봇팔은 서버에 STOP을 직접 요청할 수 있도록 항상 표시한다.
  function paintEstopRobots(){
    var el=document.getElementById('estop-robots');if(!el)return;
    var mobile=ROBOTS.map(function(r){var key='mobile-'+r.rid,picked=estopPick===key;
      return '<div class="li estop-device-row'+(r.online?' rrow':'')+'"'+(r.online?' onclick="estopPick=\''+key+'\';paintEstopRobots()"':'')+'><input type="radio" name="estop-pick" class="rpick"'+(r.online?'':' disabled')+(picked?' checked':'')+'><span class="dotm" style="width:10px;height:10px;border-radius:50%;flex:0 0 auto;background:'+kindColor(r.kind)+'"></span><div><div class="li-t">'+r.id+'</div><div class="li-s">'+r.mode+'</div></div><button class="minibtn stop"'+(r.online&&picked?'':' disabled')+' onclick="event.stopPropagation();sendStop('+r.rid+')">정지</button></div>';}).join('');
    var arms=[['left','Left Arm','왼팔'],['right','Right Arm','오른팔'],['dual','Dual Arm','양팔 동작']].map(function(a){
      var key='arm-'+a[0],picked=estopPick===key;
      return '<div class="li estop-device-row rrow" onclick="estopPick=\''+key+'\';paintEstopRobots()"><input type="radio" name="estop-pick" class="rpick"'+(picked?' checked':'')+'><span class="dotm" style="width:10px;height:10px;border-radius:50%;flex:0 0 auto;background:var(--danger)"></span><div><div class="li-t">'+a[1]+'</div><div class="li-s">'+a[2]+' 개별 정지</div></div><button class="minibtn stop"'+(picked?'':' disabled')+' onclick="event.stopPropagation();sendArmStop(\''+a[0]+'\')">정지</button></div>';
    }).join('');
    el.innerHTML=mobile+arms;
  }

  // --- 실 데이터: /api/fleet → ROBOTS/BATT 갱신 후 재렌더 (Task 4: estop 목록·배터리 파이프라인) ---
  // 전압이 기준(문서: %는 셀·컷오프 미확인 → 전압을 믿을 것). %는 6.0~8.4V 선형 근사 병기.
  var BATT_LOW_V=7.0;   // 저전압 임계(문서: /battery/voltage 7.0V 이상 권장) — 이하면 붉은색
  function battStr(v){return (v!=null)?(v.toFixed(1)+'V (~'+Math.max(0,Math.min(100,Math.round((v-6.0)/(8.4-6.0)*100)))+'%)'):'--';}
  function battLow(v){return v!=null&&v<BATT_LOW_V;}
  function battColor(v){return v==null?'':(battLow(v)?'var(--danger)':'var(--teacher)');}   // 이하=적/이상=녹
  var FLEET=[];   // 최신 fleet 스냅샷(배터리 조회용)
  var MAPMETA=null;   // map.json(원점·해상도·크기) — pose(m)→화면% 변환용
  // 순찰 오버레이(map frame). 좌표 = 2026-07-22 텔레옵 실측 등록(waypoints.yaml 최종값, CLI 2바퀴 검증).
  // PATROL_WPS=순찰 waypoint 마커(노란 동그라미), PATROL_PATH=WP를 직선으로 잇는 루프(노란 점선, 스무쓰 렌더).
  var PATROL_WPS=[{n:8,x:-0.002,y:0.044},{n:'G',x:0.400,y:0.060},{n:'S',x:0.809,y:0.113},{n:'B',x:0.909,y:0.324},{n:9,x:1.449,y:0.013},{n:10,x:1.440,y:0.661},{n:'M',x:0.850,y:0.660},{n:7,x:0.009,y:0.533}];   // M=2026-07-30 추가(tag10→tag7 1.437m 분할)
  // 벽 우회 루프(WP + 중간 경유점). 렌더에서 닫힌 Catmull-Rom 곡선으로 부드럽게. 중간점=박스/중앙벽/동쪽구조물 회피용.
  var PATROL_PATH=[[-0.002,0.044],[0.40,0.06],[0.809,0.113],[0.92,0.32],[1.00,0.56],[1.16,0.58],[1.30,0.34],[1.449,0.013],[1.46,0.30],[1.440,0.661],[0.85,0.66],[0.38,0.62],[0.009,0.533],[0.00,0.28]];
  function paintPatrolRoute(){
    var els=document.querySelectorAll('.fmap-patrol');if(!els.length||!MAPMETA)return;   // 로봇 현황·순찰 화면 양쪽
    var W=MAPMETA.width,H=MAPMETA.height,res=MAPMETA.resolution,ox=MAPMETA.origin[0],oy=MAPMETA.origin[1];
    function pct(x,y){return [(x-ox)/(W*res)*100, 100-(y-oy)/(H*res)*100];}
    var pts=PATROL_PATH.map(function(p){return pct(p[0],p[1]);});
    // 닫힌 Catmull-Rom → 부드러운 곡선(각 세그먼트를 cubic bezier로)
    var n=pts.length, d='M'+pts[0][0].toFixed(1)+','+pts[0][1].toFixed(1);
    for(var i=0;i<n;i++){
      var p0=pts[(i-1+n)%n],p1=pts[i],p2=pts[(i+1)%n],p3=pts[(i+2)%n];
      var c1x=p1[0]+(p2[0]-p0[0])/6, c1y=p1[1]+(p2[1]-p0[1])/6;
      var c2x=p2[0]-(p3[0]-p1[0])/6, c2y=p2[1]-(p3[1]-p1[1])/6;
      d+='C'+c1x.toFixed(1)+','+c1y.toFixed(1)+' '+c2x.toFixed(1)+','+c2y.toFixed(1)+' '+p2[0].toFixed(1)+','+p2[1].toFixed(1);
    }
    d+='Z';
    var dots=PATROL_WPS.map(function(w){var q=pct(w.x,w.y);return '<circle cx="'+q[0].toFixed(1)+'" cy="'+q[1].toFixed(1)+'" r="1.6" style="fill:#ffcf5e;stroke:#0e1420;stroke-width:0.3"/>';}).join('');
    var html='<path d="'+d+'" style="fill:none;stroke:#ffcf5e;stroke-width:0.7;stroke-dasharray:2 1.5;opacity:.9;stroke-linejoin:round;stroke-linecap:round"/>'+dots;
    els.forEach(function(el){el.innerHTML=html});
  }
  // 감시 구역(ZoneA~E) 오버레이 — zoneOf()와 같은 기준(중앙 밴드 ±18% + 4분면)으로 그린다.
  function paintZoneOverlay(){
    var els=document.querySelectorAll('.fmap-zones');if(!els.length||!MAPMETA)return;
    // 구분선·중앙 밴드는 그리지 않고 이름만 표시(구역 판정 기준은 zoneOf()에 그대로 있음).
    var lbl='font-size:4.4px;font-weight:800;fill:#9aa6b4;letter-spacing:.2px';
    var h='';
    // [이름, x, y, 정렬] — 서쪽(충전소·분리수거장)은 왼쪽, 동쪽(교실1·정원)은 오른쪽 정렬
    [['충전소',5.2,91,'start'],['교실',97.2,91,'end'],['분리수거장',5.2,24,'start'],['정원',97.2,24,'end'],['교무실',28.7,57,'middle']]
      .forEach(function(z){h+='<text x="'+z[1]+'" y="'+z[2]+'" text-anchor="'+z[3]+'" style="'+lbl+'">'+z[0]+'</text>'});
    els.forEach(function(el){el.innerHTML=h});
  }
  fetch('map.json').then(function(x){return x.json();}).then(function(m){MAPMETA=m;paintMapMarkers(FLEET);paintPatrolRoute();paintZoneOverlay();}).catch(function(){});
  // 실시간 위치맵: 각 로봇 AMCL pose(heartbeat)를 실 맵 좌표로 마커. pose 없는 online 로봇은 '위치 미상'.
  function paintMapMarkers(list){
    var els=document.querySelectorAll('.fmap-markers');if(!els.length||!MAPMETA)return;   // 로봇 현황·순찰 화면 양쪽
    var w=MAPMETA.width,h=MAPMETA.height,res=MAPMETA.resolution,ox=MAPMETA.origin[0],oy=MAPMETA.origin[1];
    var nopose=[];
    var html=list.filter(function(r){return r.online;}).map(function(r){
      if(!r.pose){nopose.push(r.id);return '';}
      var left=(r.pose.x-ox)/(w*res)*100, top=100-(r.pose.y-oy)/(h*res)*100;
      return '<div class="marker '+r.kind+' live" style="left:'+left.toFixed(1)+'%;top:'+top.toFixed(1)+'%" onclick="selectRobot(\'Pinky-'+r.id+'\',\''+modeLabel(r.mode||'idle')+'\',\''+r.kind+'\')" title="Pinky-'+r.id+' ('+r.pose.x.toFixed(2)+', '+r.pose.y.toFixed(2)+')">'+r.id+'</div>';
    }).join('');
    els.forEach(function(el){el.innerHTML=html});
    var txt=nopose.length?('위치 미상: '+nopose.join(', ')):'';
    document.querySelectorAll('.fmap-nopose').forEach(function(np){np.textContent=txt});
  }
  function robotById(id){for(var i=0;i<FLEET.length;i++){if(FLEET[i].id===id)return FLEET[i];}return null;}
  // (A) 최초/현재 선택이 유효 온라인 로봇이 아니면 온라인 로봇 하나를 자동선택(상단칩만; 화면 전환 없음).
  //     사용자가 직접 고르면(userPicked) 존중해 덮어쓰지 않음.
  function autoSelectRobot(list){
    if(userPicked)return;
    var cur=selectedRobotId();
    for(var i=0;i<list.length;i++){if(list[i].id===cur&&list[i].online)return;}   // 이미 유효 온라인이면 유지
    var on=list.filter(function(r){return r.online;});
    if(!on.length)return;                        // 온라인 로봇 없으면 그대로
    var r=on[0];
    document.getElementById('tb-robot').textContent='Pinky-'+r.id;
    var m=document.getElementById('tb-mode');m.textContent=modeLabel(r.mode||'idle');
    m.className='modepill'+(r.kind==='safety'?' safety':r.kind==='idle'?' idle':'');
  }
  // 상단칩 + 각 모드화면 배터리 stat을 '현재 로봇'(tb-robot)의 실 전압/색으로 일관 갱신
  function refreshBatteries(){
    var rid=selectedRobotId();
    var r=(rid!==null)?robotById(rid):null;var v=r?r.battery_v:null;
    var s=battStr(v),c=battColor(v);
    ['tb-batt','st-batt-thome','st-batt-tfol','st-batt-shome','st-batt-shome2'].forEach(function(id){
      var e=document.getElementById(id);if(e){e.textContent=s;e.style.color=c;}});
    // 선택 로봇의 이름·위치 stat 갱신(하드코딩 Pinky-31/교무실 대체)
    var nm=(rid!==null)?('Pinky-'+rid):'--';
    var loc=(r&&r.pose)?(r.pose.x.toFixed(2)+', '+r.pose.y.toFixed(2)):((rid!==null)?'위치 미상':'--');
    ['st-robot-thome','st-robot-shome2'].forEach(function(id){var e=document.getElementById(id);if(e)e.textContent=nm;});
    // 위치는 구역까지 함께 — 'ZoneA(충전소, x, y)'. 구역 판정 불가면 좌표만.
    var zl=(r&&r.pose)?zoneOf(r.pose.x,r.pose.y):null;
    var locz=zl?('Zone'+zl+'('+ZONES[zl].name+', '+loc+')'):loc;
    ['st-loc-thome','st-loc-tfol','st-loc-shome2'].forEach(function(id){
      var e=document.getElementById(id);if(e)e.textContent=locz;});
    var z=(r&&r.pose)?zoneOf(r.pose.x,r.pose.y):null;   // 순찰 화면 '현재 구역' — 이동하면 갱신
    var e3=document.getElementById('st-zone-shome');
    if(e3)e3.textContent=z?zlabel(z):((rid!==null)?'위치 미상':'--');
    // (C) t-home 상단 모드 pill·위치를 선택 로봇 실값으로
    ['st-mode-thome','st-mode-shome2'].forEach(function(id){var me=document.getElementById(id);
      if(me)me.textContent=(r&&r.online)?modeLabel(r.mode||'idle'):(rid!==null?'오프라인':'--');});
    ['st-lastloc-thome','st-lastloc-shome2'].forEach(function(id){var le=document.getElementById(id);
      if(le)le.textContent='위치 '+loc;});
  }
  function paintFleetSummary(list){
    var els=document.querySelectorAll('.fleet-sum');if(!els.length)return;   // 홈·로봇 현황 공용
    var total=list.length,online=list.filter(function(r){return r.online;}).length;
    var busy=list.filter(function(r){return r.online&&r.kind!=='idle';}).length;
    var warn=list.filter(function(r){return battLow(r.battery_v);}).length;   // 저전압(임계 이하) 로봇 수
    var html='<div class="s"><div class="n">'+total+'</div><div class="l">전체</div></div>'
      +'<div class="s"><div class="n">'+online+'</div><div class="l">온라인</div></div>'
      +'<div class="s"><div class="n">'+busy+'</div><div class="l">임무중</div></div>'
      +'<div class="s alert"><div class="n">'+warn+'</div><div class="l">주의</div></div>';
    els.forEach(function(el){el.innerHTML=html});
  }

  // ===== 홈(종합 현황) 대시보드 — 지도 없이 로봇·작업·긴급이벤트 빠른 인지 =====
  function paintHome(list){ paintHomeOps(list); paintHomeRobots(list); }   // 긴급알림 패널 제거(경고·고장 타일이 이벤트로 링크)
  // 전체 운영 현황 (구성안 §3). 없는 신호(충전·장애·금일임무·수동개입)는 '—'. 끊김·경고는 강조색.
  function paintHomeOps(list){
    var el=document.getElementById('home-ops'); if(!el)return;
    list=list||[];
    var total=list.length;
    var on=list.filter(function(r){return r.online;});
    var offline=total-on.length;                                             // 통신 끊김
    var idle=on.filter(function(r){return !r.mode||r.mode==='idle';}).length;
    var busy=on.filter(function(r){return r.mode==='patrol'||r.mode==='assist';}).length;   // 임무 수행
    var warn=on.filter(function(r){return battLow(r.battery_v);}).length;    // 경고(배터리 부족)
    var bv=on.filter(function(r){return r.battery_v!=null;}).map(function(r){return r.battery_v;});
    var avg=bv.length?(bv.reduce(function(a,b){return a+b;},0)/bv.length):null;
    var online=on.length;
    var charging=chargingCount(list);                                        // 전압 상승 = 충전 중
    var t0=opsT0();                                                          // 기준 시점 이후만 카운트(히스토리 불변)
    var evNew=EVLOG.filter(function(e){return (e.t||'')>t0;}).length;        // 이벤트: 초기화 이후 발생분
    var alNew=EVLOG.filter(function(e){var s=(e.type||'')+' '+(e.result||'')+' '+(e.status||'');
      return /fire|화재|경고|긴급정지/.test(s)&&!/해소|해제/.test(s)&&(e.t||'')>t0;}).length;   // 알림: 초기화 이후 활성
    var h=new Date().getHours(); var zone=(h>=9&&h<18)?'수업 보조':'순찰 당직';
    // 2개 항목씩 하나의 카드로 묶음(총 4카드). 각 항목 클릭 시 연관 화면.
    function item(label,val,cls,go){return '<div class="opsitem clk '+(cls||'')+'" onclick="'+go+'"><div class="n">'+val+'</div><div class="l">'+label+'</div></div>';}
    function card(a,b){return '<div class="opscard">'+a+'<div class="opsslash">/</div>'+b+'</div>';}
    el.innerHTML=
      '<div class="opsgrid">'
      +card(item('전체',total,'',"nav('fleet')"),      item('온라인',online,'',"nav('fleet')"))
      +card(item('응답 없음',offline,offline>0?'danger':'',"nav('fleet')"), item('고장',0,'',"nav('events')"))
      +card(item('알림',alNew,alNew>0?'danger':'',"nav('noti')"), item('이벤트',evNew,'',"nav('events')"))
      +'</div>';
  }
  // ④ 긴급 알림·이벤트 — 활성 화재 + 저전압, 없으면 최근 이벤트
  function paintHomeAlerts(list){
    var el=document.getElementById('home-alerts'); if(!el)return;
    var rows=[];
    EVLOG.filter(function(e){var s=(e.type||'')+' '+(e.result||'')+' '+(e.status||'');return /fire|화재|경고|긴급정지/.test(s)&&!/해소|해제/.test(s);})
      .slice(0,3).forEach(function(e){
        rows.push('<div class="li"><span class="ic" data-ic="alert" style="color:#f97316"></span>'
          +'<div><div class="li-t">화재 감지 — '+esc(e.pos||e.result||'')+'</div>'
          +'<div class="li-s">'+esc(evRobotLabel(e.robot)||'천장CCTV')+'</div></div>'
          +'<span class="li-x pill alert nodot">경고</span></div>'); });
    (list||[]).filter(function(r){return r.online&&battLow(r.battery_v);}).forEach(function(r){
      rows.push('<div class="li"><span class="ic" data-ic="battery" style="color:var(--danger)"></span>'
        +'<div><div class="li-t">Pinky-'+r.id+' 배터리 주의</div><div class="li-s">'+battStr(r.battery_v)+'</div></div>'
        +'<span class="li-x pill alert nodot">주의</span></div>'); });
    if(rows.length){ el.innerHTML=rows.join(''); paintIcons(el); return; }
    var recent=EVLOG.slice(0,3);
    if(!recent.length){ el.innerHTML='<div class="li"><div class="li-t">긴급 알림 없음</div></div>'; return; }
    el.innerHTML='<div class="li"><div class="li-s" style="color:var(--muted)">긴급 알림 없음 · 최근 이벤트</div></div>'
      +recent.map(function(e){return '<div class="li"><div><div class="li-t">'+esc(evTypeLabel(e.type||''))+'</div>'
        +'<div class="li-s">'+esc((evRobotLabel(e.robot)||'')+' · '+String(e.t||'').slice(5,16))+'</div></div></div>';}).join('');
  }
  // ⑤ 로봇 현황(컴팩트) — 이름·상태·배터리, 클릭 시 선택
  function paintHomeRobots(list){
    var el=document.getElementById('home-robots'); if(!el)return;
    if(!list||!list.length){ el.innerHTML='<div class="li"><div class="li-t">로봇 없음</div></div>'; return; }
    el.innerHTML=list.map(function(r){
      var bcls=(r.battery_v==null)?'idle':(battLow(r.battery_v)?'alert':'ok');
      var dis=r.online?'':' disabled';                                      // 오프라인은 명령 불가
      return '<div class="li">'
        +'<span class="dotm" style="background:'+(r.online?'var(--teacher)':'var(--muted)')+'"></span>'
        +'<div style="flex:1"><div class="li-t">Pinky-'+r.id+'</div><div class="li-s">'+(r.online?modeLabel(r.mode||'idle'):'오프라인')+'</div></div>'
        +'<div class="rowbtns">'
        +'<button class="minibtn teacher" onclick="rowMode('+r.id+",'teacher')\""+dis+'>수업 보조</button>'
        +'<button class="minibtn safety" onclick="rowMode('+r.id+",'safety')\""+dis+'>순찰 당직</button>'
        +'</div>'
        +'<span class="li-x pill '+bcls+' nodot">'+battStr(r.battery_v)+'</span></div>';
    }).join('');
  }
  // 홈 로봇 행 버튼 — 해당 로봇 선택 후 그 모드 페이지로 바로 이동.
  function rowMode(id,mode){
    document.getElementById('tb-robot').textContent='Pinky-'+id; userPicked=true; userCleared=false;
    refreshBatteries();
    if(mode==='teacher')enterTeacher(); else enterSafety();
  }
  function paintFleetList(list){
    var el=document.getElementById('fleet-list');if(!el)return;
    el.innerHTML=list.map(function(r){
      var kind=r.online?r.kind:'idle';
      var bcls=(r.battery_v==null)?'idle':(battLow(r.battery_v)?'alert':'ok');   // 이상=ok(녹)/이하=alert(적)
      var dis=r.online?'':' disabled';                                        // 오프라인은 재측위 불가
      return '<div class="li rrow" onclick="selectRobot(\'Pinky-'+r.id+'\',\''+modeLabel(r.mode||'idle')+'\',\''+kind+'\')">'
        +'<span class="dotm" style="background:'+(r.online?'var(--teacher)':'var(--muted)')+'"></span><div style="flex:1"><div class="li-t">Pinky-'+r.id
        +' · '+(r.online?modeLabel(r.mode||'idle'):'오프라인')+'</div><div class="li-s">'+(r.ip||'-')+' · WASAB_ROBOT_ID '+r.id+' · ROS_DOMAIN_ID '+(r.domain!=null?r.domain:'-')+'</div></div>'
        +'<button class="minibtn reloc" style="margin-right:6px"'+dis+' onclick="event.stopPropagation();sendRelocalize('+r.id+')">재측위</button>'
        +'<button class="minibtn agent '+(r.online?'on':'off')+'" style="margin-right:8px" onclick="event.stopPropagation();toggleAgent('+r.id+','+(r.online?'true':'false')+')">에이전트 '+(r.online?'끄기':'켜기')+'</button>'
        +'<span class="li-x pill '+bcls+' nodot">'+battStr(r.battery_v)+'</span></div>';
    }).join('');
  }
  // 순찰 화면 '가용 로봇 현황' — 로봇 현황과 같은 정보로, 온라인 + 대기(idle) 로봇만.
  // 클릭하면 화면 이동 없이 순찰에 쓸 로봇으로 선택된다.
  function paintPatrolRobots(list){
    var els=document.querySelectorAll('.avail-robots');if(!els.length)return;   // 순찰·수업 보조 두 화면
    var cur=selectedRobotId();
    // 온라인 로봇은 모두 보여주되, 임무 중(assist/patrol)인 로봇은 선택 불가로 표시한다.
    var on=(list||[]).filter(function(r){return r.online;});
    var free=on.filter(function(r){return !r.mode||r.mode==='idle';});   // 대기 = 선택 가능
    if(!on.length){var none='<div class="li"><div><div class="li-t">가용 로봇 없음</div>'
      +'<div class="li-s">온라인 로봇이 없습니다</div></div></div>';
      els.forEach(function(el){el.innerHTML=none});return;}
    // 자동선택은 '유효한 선택이 없을 때'만 — 이미 온라인 로봇이 선택돼 있으면 덮지 않는다
    // (순찰 중 로봇을 골라둔 걸 대기 로봇으로 덮어써 정지 못 하던 버그 방지).
    var curOnline=(cur!==null)&&on.some(function(r){return r.id===cur;});
    if(free.length===1&&!curOnline&&!userCleared){setRobotChip(free[0]);cur=free[0].id;}
    var html=on.map(function(r){
      // 수업 보조(assist) 중인 로봇만 선택 불가. ★순찰(patrol) 중은 '정지'를 위해 선택 가능해야 함.
      var busy=(r.mode==='assist'), sel=(!busy&&r.id===cur);
      var bcls=(r.battery_v==null)?'idle':(battLow(r.battery_v)?'alert':'ok');
      return '<div class="li'+(busy?' busy':' rrow')+'"'
        +(busy?'':' onclick="pickPatrolRobot('+r.id+',\''+modeLabel(r.mode||'idle')+'\',\''+r.kind+'\')"')
        +(sel?' style="background:var(--primary-soft);border-radius:10px"':'')+'>'
        +'<input type="radio" class="rpick"'+(sel?' checked':'')+(busy?' disabled':'')+'>'   // name은 카드별로 부여(아래)
        +'<div><div class="li-t">Pinky-'+r.id+' · '+modeLabel(r.mode||'idle')+'</div>'
        +'<div class="li-s">'+(r.ip||'-')+' · WASAB_ROBOT_ID '+r.id+' · ROS_DOMAIN_ID '+(r.domain!=null?r.domain:'-')+'</div></div>'
        +(busy?'<span class="li-x pill run nodot" style="margin-left:auto">임무 중</span>'
              :(sel?'<span class="li-x pill run nodot" style="margin-left:auto">선택됨</span>':''))
        +'<span class="li-x pill '+bcls+' nodot">'+battStr(r.battery_v)+'</span></div>';
    }).join('');
    // 카드가 둘(순찰·수업 보조)이라 라디오 그룹 이름을 카드별로 나눈다.
    // 같은 name이면 브라우저가 마지막 카드 하나만 체크 상태로 유지해, 다른 카드는 영영 체크 안 됨.
    els.forEach(function(el,i){
      el.innerHTML=html;
      el.querySelectorAll('.rpick').forEach(function(rd){rd.name='robot-pick-'+i;});
    });
  }
  // 상단 로봇칩만 갱신(재렌더 없음) — 자동 선택에서 재귀 방지용.
  function setRobotChip(r){
    document.getElementById('tb-robot').textContent='Pinky-'+r.id;
    var m=document.getElementById('tb-mode');m.textContent=modeLabel(r.mode||'idle');
    m.className='modepill'+(r.kind==='safety'?' safety':r.kind==='idle'?' idle':'');
  }
  // 카드에서 선택/해제 — 같은 줄을 다시 누르면 선택 해제(다른 화면에서 고를 수 있게).
  // selectRobot과 달리 화면을 옮기지 않는다(그 화면에 머문 채 시작 버튼으로).
  var userCleared=false;   // 사용자가 명시적으로 해제함 → 단일 로봇 자동선택을 막는다
  function pickPatrolRobot(id,label,kind){
    userPicked=true;
    var m=document.getElementById('tb-mode');
    if(selectedRobotId()===id){                       // 토글 해제
      userCleared=true;
      document.getElementById('tb-robot').textContent='--';
      m.textContent='--';m.className='modepill idle';
    }else{
      userCleared=false;
      document.getElementById('tb-robot').textContent='Pinky-'+id;
      m.textContent=label;
      m.className='modepill'+(kind==='safety'?' safety':kind==='idle'?' idle':'');
    }
    refreshBatteries();refreshPatrolUI();paintPatrolRobots(FLEET);
  }
  // 충전 판정 — heartbeat엔 충전 플래그가 없어 전압 추세로 추정.
  // 전압 상승 = 충전 중 / 하강·변동없음 = 충전 안 함. 최근 창(90s) 앞1/3 vs 뒤1/3 평균 비교.
  var BATTHIST={};                                   // id -> [{t,v}]
  var CHG_WINDOW_MS=90000, CHG_MIN_SPAN_MS=40000, CHG_RISE_V=0.02;
  function recordBatt(list){
    var now=Date.now();
    (list||[]).forEach(function(r){
      if(!r.online||r.battery_v==null)return;
      var h=BATTHIST[r.id]||(BATTHIST[r.id]=[]);
      h.push({t:now,v:r.battery_v});
      while(h.length&&now-h[0].t>CHG_WINDOW_MS)h.shift();
    });
  }
  function isCharging(id){
    var h=BATTHIST[id]; if(!h||h.length<4)return false;
    if(h[h.length-1].t-h[0].t<CHG_MIN_SPAN_MS)return false;   // 충분한 관측 시간 확보 전엔 판정 보류
    var k=Math.max(1,Math.floor(h.length/3));
    function avg(a){return a.reduce(function(s,x){return s+x.v;},0)/a.length;}
    return (avg(h.slice(-k))-avg(h.slice(0,k)))>=CHG_RISE_V;   // 뒤 평균이 앞보다 오르면 충전
  }
  function chargingCount(list){return (list||[]).filter(function(r){return r.online&&isCharging(r.id);}).length;}

  function applyFleet(list){
    FLEET=list;
    recordBatt(list);   // 충전 판정용 전압 이력 적재
    list.forEach(function(r){if(r.name)EVIDMAP[r.name]='Pinky-'+r.id;});   // 이벤트 출력용 name→Pinky-<id> 매핑(robots.yaml 기반)
    if(evLoaded)evBoot(list);   // 서버 이력 로드 후 최초 fleet에서 등록 로봇 상태 초기 기록
    watchFleetEvents(list);   // (A) 로봇 실이벤트(모드/온오프라인) 자동 기록
    ROBOTS=list.map(function(r){return {id:'Pinky-'+r.id,rid:r.id,raw:(r.mode||'idle'),online:!!r.online,mode:(r.online?modeLabel(r.mode||'idle'):'오프라인'),kind:(r.online?r.kind:'idle')};});
    BATT={};
    list.forEach(function(r){BATT['Pinky-'+r.id]=battStr(r.battery_v);});
    if(document.getElementById('estop-robots'))paintEstopRobots();
    // 도킹 상태(dock_event)가 바뀌면 수업 동행 화면 문구도 즉시 갱신
    var tf=document.getElementById('t-following');
    if(tf&&tf.classList.contains('active'))applyAssistUI();
    paintFleetSummary(list);paintFleetList(list);paintMapMarkers(list);paintPatrolRobots(list);paintHome(list);
    autoSelectRobot(list);   // (A) 최초 온라인 로봇 자동선택 → 상단칩이 실 로봇 대상
    refreshBatteries();   // 상단칩·화면 stat 실시간 반영
    var tr=document.getElementById('t-returning');if(tr&&tr.classList.contains('active'))updateReturning();
  }
  function loadFleet(){fetch('/api/fleet').then(function(x){return x.json();}).then(applyFleet).catch(function(){});}

  // --- 인증 후에만 도는 런타임(WS /ws/state + 폴링 폴백). 로그아웃/세션소실 시 정지. ---
  var runtimeActive = false, _ws = null, _pollTimer = null;
  function _startPoll(){ if(!_pollTimer) _pollTimer = setInterval(loadFleet, 2000); }
  function _stopPoll(){ if(_pollTimer){ clearInterval(_pollTimer); _pollTimer = null; } }
  function _connect(){
    if (!runtimeActive) return;
    var ws;
    try { ws = new WebSocket((location.protocol==='https:'?'wss':'ws')+'://'+location.host+'/ws/state'); }
    catch (err) { _startPoll(); setTimeout(_connect, 3000); return; }
    _ws = ws;
    ws.onopen = function(){ _stopPoll(); };
    ws.onmessage = function(e){ try { applyFleet(JSON.parse(e.data)); } catch (x) {} };
    ws.onclose = function(){ if(!runtimeActive) return; _startPoll(); setTimeout(_connect, 3000); };
    ws.onerror = function(){ try { ws.close(); } catch (x) {} };
  }
  function startAuthenticatedRuntime(){
    if (runtimeActive) return;          // 재로그인 시 중복 방지
    runtimeActive = true;
    loadFleet(); evLoadServer(); _connect(); startArmPrompts();
  }
  function stopAuthenticatedRuntime(){
    runtimeActive = false;
    _stopPoll();
    stopArmPrompts();
    if (_ws){ try { _ws.close(); } catch (e) {} _ws = null; }
  }

  // --- 세션 라우팅(로그인/로그아웃) + 계정관리 ---
  function _applyRole(role){
    window.__role = role;
    var m = document.getElementById('menu-accounts'); if (m) m.style.display = (role==='admin') ? '' : 'none';
    var arm = document.getElementById('menu-arm-test'); if (arm) arm.style.display = (role==='admin') ? '' : 'none';
  }
  function _applyGreeting(name){
    window.__account = name || '';
    document.querySelectorAll('.greet-name').forEach(function(el){ el.textContent = window.__account; });   // entry·fleet 등 여러 곳
    var na = document.getElementById('nav-account'); if (na) na.textContent = window.__account;
  }
  async function bootAuth(){
    try {
      var s = await (await fetch('/api/session')).json();
      if (s.authenticated){ _applyRole(s.role); _applyGreeting(s.id); nav('entry'); startAuthenticatedRuntime(); }   // 첫 화면 = 종합 현황(홈)
      else { show('login'); }
    } catch (e) { show('login'); }
  }

  // 얼굴 로그인 — 웹캠 한 장을 /api/login/face 로 보내 판정(비밀번호와 OR).
  // 판정은 face-recog 서비스가 하고, 성공 시 처리는 doLogin 과 동일 경로를 탄다.
  var faceStream=null, faceTimer=null, faceTries=0;
  var FACE_MAX_TRIES=5, FACE_INTERVAL_MS=1500;
  function faceMsg(t,show,bad){var el=document.getElementById('face-msg');if(!el)return;
    el.textContent=t||'';el.style.display=show?'block':'none';
    el.className='facemsg'+(bad?' bad':'')}
  function faceScan(on,text){                 // 썬글라스 와사비 스캔 연출 on/off
    var s=document.getElementById('face-scan');if(!s)return;
    s.classList.toggle('on',!!on);
    if(!on)s.classList.remove('ok');
    var t=document.getElementById('face-scantext');
    if(t&&text)t.innerHTML=text;
  }
  function stopFaceCam(){
    if(faceTimer){clearInterval(faceTimer);faceTimer=null}
    if(faceStream){faceStream.getTracks().forEach(function(t){t.stop()});faceStream=null}
    var v=document.getElementById('face-cam');if(v){v.srcObject=null;v.style.display='none'}
    var b=document.getElementById('face-btn');if(b)b.disabled=false;
  }
  function grabJpeg(v){                       // 현재 프레임 → JPEG blob
    var c=document.createElement('canvas');
    c.width=v.videoWidth||640;c.height=v.videoHeight||480;
    c.getContext('2d').drawImage(v,0,0,c.width,c.height);
    return new Promise(function(res){c.toBlob(res,'image/jpeg',0.85)});
  }
  // 카메라 선택 규칙
  //  ★천장 CCTV(콘솔 관제 전용)는 절대 사용하지 않는다. 콘솔이 켜져 있으면 V4L2 배타 점유라
  //    애초에 열리지 않지만, 콘솔이 꺼진 사이 브라우저가 선점하면 콘솔 기동을 막게 되므로
  //    라벨로 한 번 더 차단하고, 잘못 열렸으면 즉시 반납한다.
  var FACE_CAM_DENY=/USB Camera/i;      // = 천장 CCTV (/dev/video0)
  var FACE_CAM_PREFER=/SNAP/i;          // = 얼굴 인증용 카메라 (SNAP U2)
  function _camLabel(stream){
    var t=stream&&stream.getVideoTracks&&stream.getVideoTracks()[0];
    return (t&&t.label)||'';
  }
  function _release(stream){
    if(stream)stream.getTracks().forEach(function(t){t.stop()});
  }
  async function openFaceCamera(){
    var base={width:640,height:480};
    var devs=[];
    try{ devs=(await navigator.mediaDevices.enumerateDevices())
              .filter(function(d){return d.kind==='videoinput'}); }catch(e){}
    // 라벨을 아는 경우(권한 승인 후): 선호 장치 우선, 천장 CCTV 는 후보에서 제외
    var named=devs.filter(function(d){return d.label&&!FACE_CAM_DENY.test(d.label)});
    named.sort(function(a,b){
      return (FACE_CAM_PREFER.test(b.label)?1:0)-(FACE_CAM_PREFER.test(a.label)?1:0)});
    for(var i=0;i<named.length;i++){
      try{
        return await navigator.mediaDevices.getUserMedia(
          {video:Object.assign({deviceId:{exact:named[i].deviceId}},base)});
      }catch(e){}
    }
    // 라벨을 모르는 첫 시도(권한 전): 열어보고 천장 CCTV 면 즉시 반납 후 재선택.
    // 기본 장치가 점유(콘솔 사용 중)라 실패해도 포기하지 않고 아래에서 장치별로 재시도한다.
    var s=null;
    try{ s=await navigator.mediaDevices.getUserMedia({video:base}); }catch(e){ s=null; }
    if(s&&!FACE_CAM_DENY.test(_camLabel(s)))return s;
    _release(s);                                    // ★천장 CCTV 반납 — 콘솔 몫
    try{ devs=(await navigator.mediaDevices.enumerateDevices())
              .filter(function(d){return d.kind==='videoinput'&&!FACE_CAM_DENY.test(d.label)}); }
    catch(e){ devs=[]; }
    for(var j=0;j<devs.length;j++){
      try{
        return await navigator.mediaDevices.getUserMedia(
          {video:Object.assign({deviceId:{exact:devs[j].deviceId}},base)});
      }catch(e){}
    }
    return null;
  }
  async function faceLogin(){
    if(faceTimer)return;                      // 이미 시도 중
    var v=document.getElementById('face-cam'),btn=document.getElementById('face-btn');
    faceStream=await openFaceCamera();
    if(!faceStream){
      faceMsg('사용 가능한 카메라가 없습니다. (천장 CCTV는 관제 전용이라 사용하지 않습니다)',
              true,true);return}
    v.srcObject=faceStream;v.style.display='block';
    try{await v.play()}catch(e){}
    if(btn)btn.disabled=true;
    faceTries=0;faceMsg('',false);
    faceScan(true,'얼굴 스캔 중<span class="dots"><i>.</i><i>.</i><i>.</i></span>');
    faceTimer=setInterval(async function(){
      faceTries++;
      var blob=await grabJpeg(v);
      var r;
      try{
        r=await fetch('/api/login/face',{method:'POST',
          headers:{'Content-Type':'image/jpeg'},body:blob});
      }catch(e){stopFaceCam();faceScan(false);faceMsg('서버에 연결할 수 없습니다.',true,true);return}
      if(r.status===200){
        var b=await r.json();stopFaceCam();faceMsg('',false);
        var s=document.getElementById('face-scan');if(s)s.classList.add('ok');
        faceScan(true,'환영합니다, '+b.id+' 선생님!');     // 성공 연출 잠깐 보여주고 입장
        _applyRole(b.role);_applyGreeting(b.id);
        logAuthEvent('로그인(얼굴)',b.id,'성공');
        setTimeout(function(){faceScan(false);nav('entry');startAuthenticatedRuntime()},900);
        return;
      }
      if(r.status===503){stopFaceCam();faceScan(false);
        faceMsg('얼굴 인증 서비스가 준비되지 않았습니다.',true,true);return}
      if(faceTries>=FACE_MAX_TRIES){stopFaceCam();faceScan(false);
        faceMsg('얼굴을 인식하지 못했습니다. 아이디·비밀번호로 로그인해 주세요.',true,true);return}
      faceScan(true,'얼굴 스캔 중 ('+faceTries+'/'+FACE_MAX_TRIES+')<span class="dots"><i>.</i><i>.</i><i>.</i></span>');
    },FACE_INTERVAL_MS);
  }

  async function doLogin(){
    var id = document.getElementById('login-id').value;
    var pw = document.getElementById('login-pw').value;
    var err = document.getElementById('login-err');
    var r = await fetch('/api/login', {method:'POST', headers:{'Content-Type':'application/json'},
                                       body: JSON.stringify({id:id, password:pw})});
    if (r.status === 200){
      var b = await r.json(); _applyRole(b.role); _applyGreeting(b.id);
      logAuthEvent('로그인',b.id,'성공');
      document.getElementById('login-pw').value = '';
      nav('entry'); startAuthenticatedRuntime();   // 로그인 후 첫 화면 = 종합 현황(홈)
    } else {
      err.textContent = '아이디 또는 비밀번호가 올바르지 않습니다.'; err.style.display = 'block';
    }
  }

  // ── 얼굴 인증 게이트 — 인증 전에는 지시(버튼·모션)를 막는다. 상태 근거는 서버 세션 ──
  var faceVerified=false;
  function applyFaceGate(){                       // 배너 표시 + 지시 UI 활성/비활성
    var g=document.getElementById('f-gate');if(g)g.style.display=faceVerified?'none':'block';
    document.querySelectorAll('#t-home [data-face-command], #mo-start')
      .forEach(function(b){b.disabled=!faceVerified});
  }
  async function refreshFaceGate(){               // 세션에서 인증 상태 조회
    try{ var r=await fetch('/api/session'); var s=await r.json();
         faceVerified=!!s.face_verified; }catch(e){ faceVerified=false }
    applyFaceGate();
  }
  async function verifyFace(){                    // 웹캠 1장으로 인증 승격
    var btn=document.getElementById('f-gate-btn'),msg=document.getElementById('f-gate-msg');
    var s=await openFaceCamera();
    if(!s){msg.textContent='사용 가능한 카메라가 없습니다.';return}
    btn.disabled=true;msg.textContent='얼굴 확인 중…';
    var v=document.createElement('video');v.srcObject=s;v.muted=true;v.playsInline=true;
    try{await v.play()}catch(e){}
    await new Promise(function(r){setTimeout(r,800)});   // 카메라 노출 안정화
    var ok=false;
    for(var i=0;i<5&&!ok;i++){
      var blob=await grabJpeg(v);
      try{
        var r=await fetch('/api/face/verify',{method:'POST',
              headers:{'Content-Type':'image/jpeg'},body:blob});
        if(r.status===200){var b=await r.json();ok=true;
          msg.textContent='';faceVerified=true;applyFaceGate();
          logAuthEvent('얼굴 인증',b.face_name,'통과');}
        else if(r.status===503){msg.textContent='얼굴 인증 서비스가 준비되지 않았습니다.';break}
      }catch(e){msg.textContent='서버에 연결할 수 없습니다.';break}
      if(!ok)await new Promise(function(r){setTimeout(r,700)});
    }
    if(!ok&&!msg.textContent)msg.textContent='얼굴을 인식하지 못했습니다. 다시 시도해 주세요.';
    s.getTracks().forEach(function(t){t.stop()});
    btn.disabled=false;
  }

  // ── 지시 명령 — 버튼과 모션이 같은 함수를 호출한다(입력수단만 다름) ──
  // 실제 로봇 발행은 여기 한 곳에만 둔다. 추종 주행·인사는 아직 미구현이라 기록만 남긴다.
  var CLASSROOM_TAG=9;                                   // 교실(ZoneB) 도킹 태그
  var CMD_LABEL={CLASSROOM:'교실이동',START:'추종 시작',PAUSE:'일시정지',
                 HOME:'홈복귀',GREET:'인사'};
  function sendCommand(cmd,via,who){
    var label=CMD_LABEL[cmd]||cmd;
    var id=selectedRobotId();
    if(id===null){alert('먼저 로봇을 선택하세요.');return false}
    var note='';
    // 버튼: 교실(tag9) 도킹 + 화면 전환. ★모드(cmd_mode)는 건드리지 않는다 —
    // assist 로 바꾸면 도킹이 끝나도 '임무중'으로 남아 가용 로봇 목록에서 빠지고,
    // 그러면 다음 지시를 아예 못 보낸다(2026-07-27 실기에서 발생).
    if(cmd==='CLASSROOM'){ sendDockTag(CLASSROOM_TAG); assistOn=true; show('t-following'); }
    else if(cmd==='START'){ startAssist(); }               // 모션 ☝️: 선생님 추종 시작
    else if(cmd==='PAUSE'){ sendMode('idle'); }            // 대기로 (주행 멈춤)
    else if(cmd==='HOME'){ sendDock(); show('t-returning'); }  // 홈 태그로 도킹(검증된 경로)
    else if(cmd==='GREET'){ note='(팔 동작 미구현)'; }      // 팔 제어는 별도 담당
    else { return false }
    logAuthEvent('지시 · '+label,(who? who+' · ':'')+(via==='motion'?'모션':'버튼'),
                 note||'발행');
    return true;
  }

  // ── 모션 캡쳐 — 얼굴 인증(누구) AND 손동작 제스처. 판정은 /api/face/identify(서버) ──
  // 카메라 열기·프레임 캡쳐는 얼굴 로그인과 같은 함수를 재사용한다(천장 CCTV 차단 규칙 포함).
  var moStream=null, moTimer=null, moLastGesture=null;
  var MO_INTERVAL_MS=900;
  var GESTURE_LABEL={START:'추종 시작',HOME:'홈복귀',PAUSE:'일시정지',
                     GREET:'인사',MOVE_OBJECT:'물체 이동'};
  function moSet(id,text,cls){var el=document.getElementById(id);if(!el)return;
    el.textContent=text;if(cls)el.className=cls}
  async function startMotion(){
    if(moTimer)return;
    var v=document.getElementById('mo-cam');
    moStream=await openFaceCamera();
    if(!moStream){moSet('mo-hint','사용 가능한 카메라가 없습니다. (천장 CCTV는 관제 전용)');return}
    v.srcObject=moStream;v.style.display='block';
    try{await v.play()}catch(e){}
    document.getElementById('mo-start').disabled=true;
    document.getElementById('mo-stop').disabled=false;
    moLastGesture=null;
    moSet('mo-hint','선생님을 확인하는 중…');
    moTimer=setInterval(async function(){
      var blob=await grabJpeg(v),r;
      try{ r=await fetch('/api/face/identify',{method:'POST',
             headers:{'Content-Type':'image/jpeg'},body:blob}); }
      catch(e){ moSet('mo-hint','서버에 연결할 수 없습니다.'); return }
      if(!r.ok){ moSet('mo-hint', r.status===503?'얼굴 인증 서비스가 준비되지 않았습니다.'
                                                :'판정 실패('+r.status+')'); return }
      var d=await r.json();
      if(d.name){                                  // 얼굴 인증 통과
        moSet('mo-who',d.name+' 확인','pill ok');
        if(d.gesture){                             // 제스처 확정된 순간에만 값이 온다
          var label=GESTURE_LABEL[d.gesture]||d.gesture;
          moSet('mo-gesture','제스처 '+label,'pill run nodot');
          moSet('mo-hint','인식: '+label);
          if(d.gesture!==moLastGesture){           // 같은 동작 반복 실행 방지
            moLastGesture=d.gesture;
            sendCommand(d.gesture,'motion',d.name);   // 버튼과 동일 경로로 지시 + 이벤트 등록
          }
        }
      }else{                                       // 미인증 → 제스처는 무시(AND 조건)
        moSet('mo-who','인증 대기','pill idle');
        moSet('mo-gesture','제스처 —','pill idle nodot');
        moSet('mo-hint','등록된 선생님이 확인되어야 모션이 인식됩니다.');
      }
    },MO_INTERVAL_MS);
  }
  function stopMotion(){
    if(moTimer){clearInterval(moTimer);moTimer=null}
    if(moStream){moStream.getTracks().forEach(function(t){t.stop()});moStream=null}
    var v=document.getElementById('mo-cam');if(v){v.srcObject=null;v.style.display='none'}
    var a=document.getElementById('mo-start'),b=document.getElementById('mo-stop');
    if(a)a.disabled=false;if(b)b.disabled=true;
    moSet('mo-who','인증 대기','pill idle');moSet('mo-gesture','제스처 —','pill idle nodot');
    moSet('mo-hint','시작을 누르면 카메라로 선생님을 확인한 뒤 손동작을 인식합니다.');
  }

  // 인증 이벤트 — 로봇이 아닌 웹앱 행위라 robot/ip/pos 는 고정값으로 넣는다.
  function logAuthEvent(type,who,result){
    logEvent(type,who||'-',result||'-',{robot:'웹앱',ip:'',mode:'',pos:''});
  }

  async function doLogout(){
    var who=window.__account||'-';
    stopAuthenticatedRuntime();
    await fetch('/api/logout', {method:'POST'});
    logAuthEvent('로그아웃',who,'완료');
    show('login');
  }

  async function openAccounts(){ show('accounts'); await loadTeachers(); }

  async function loadTeachers(){
    var list = document.getElementById('acc-list');
    var r = await fetch('/api/teachers');
    if (r.status === 403){ list.textContent = '관리자만 접근 가능합니다.'; return; }
    if (!r.ok){ return; }
    var rows = await r.json();
    list.innerHTML = rows.map(function(t){
      var rl = (t.role==='admin') ? '관리자' : '일반 유저';
      return '<div class="accrow">'+esc(t.id)+' <span class="sub">'+rl+'</span>'+
             (t.role==='teacher' ? ' <button onclick="deleteTeacher(\''+esc(t.id)+'\')">삭제</button>' : '')+
             '</div>';
    }).join('');
  }

  async function createTeacher(){
    var id = document.getElementById('acc-id').value;
    var pw = document.getElementById('acc-pw').value;
    var role = document.getElementById('acc-role').value;
    var msg = document.getElementById('acc-msg');
    var r = await fetch('/api/teachers', {method:'POST', headers:{'Content-Type':'application/json'},
                                          body: JSON.stringify({id:id, password:pw, role:role})});
    if (r.status === 200){ document.getElementById('acc-id').value=''; document.getElementById('acc-pw').value=''; msg.style.display='none'; loadTeachers(); }
    else { msg.textContent = (r.status===409?'이미 있는 아이디':'아이디/비밀번호 형식 오류'); msg.style.display='block'; }
  }

  async function deleteTeacher(id){
    if (!confirm(id+' 계정을 삭제할까요?')) return;
    await fetch('/api/teachers/'+encodeURIComponent(id), {method:'DELETE'});
    loadTeachers();
  }

  function estopFetch(target,active){return fetch('/api/cmd/estop',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({target:target,active:active})}).then(function(x){if(!x.ok)throw new Error('estop '+x.status);});}
  function doEstopAll(){closeEstop();var m=document.getElementById('tb-mode');m.textContent='정지됨';m.className='modepill alert';
    logEvent('긴급정지','전 로봇 즉시 정지','정지',{robot:'전체',ip:'-',mode:'긴급정지',pos:'-'});
    estopFetch('all',true).catch(function(e){alert('전 로봇 정지 발행 실패: '+e.message)});}
  function doReleaseAll(){closeEstop();var m=document.getElementById('tb-mode');m.textContent='대기';m.className='modepill idle';
    logEvent('긴급정지 해제','전 로봇 대기 복귀','해제',{robot:'전체',ip:'-',mode:'긴급정지',pos:'-'});
    estopFetch('all',false).catch(function(e){alert('해제 발행 실패: '+e.message)});}
  // doEstopOne 제거 — 개별 로봇 정지 미지원(spec §10.6). 로봇측 target 필터 구현 시 backend 연동과 함께 재도입.

  function tickClock(){var el=document.getElementById('fclock');if(el){var d=new Date();el.textContent='실시간 '+d.toTimeString().slice(0,8)}}
  setInterval(tickClock,1000);tickClock();
  paintIcons();
  paintZoneLegend();
  bootAuth();   // 세션 확인 후 로그인/entry 라우팅(+인증 시에만 런타임 시작)

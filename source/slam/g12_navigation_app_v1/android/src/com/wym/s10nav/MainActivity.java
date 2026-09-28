package com.wym.s10nav;

import android.app.*;
import android.os.*;
import android.graphics.*;
import android.graphics.drawable.GradientDrawable;
import android.view.*;
import android.widget.*;
import org.json.*;
import com.skydroid.rcsdk.*;
import com.skydroid.rcsdk.key.RemoteControllerKey;
import com.skydroid.rcsdk.common.callback.CompletionCallbackWith;
import com.skydroid.rcsdk.common.error.SkyException;
import java.util.function.Consumer;
import java.io.*;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.concurrent.*;
import javax.crypto.Mac;
import javax.crypto.spec.SecretKeySpec;

/** Operator UI only: all localization and navigation velocities are computed on Orin. */
public final class MainActivity extends Activity {
    static final int BG=Color.rgb(10,19,30), CARD=Color.rgb(24,40,56), WHITE=Color.rgb(224,238,244), TEAL=Color.rgb(71,213,191), GOLD=Color.rgb(255,189,91);
    final Handler ui=new Handler(Looper.getMainLooper());
    final ScheduledExecutorService io=Executors.newSingleThreadScheduledExecutor();
    final ScheduledExecutorService rcIo=Executors.newSingleThreadScheduledExecutor();
    final ExecutorService commands=Executors.newSingleThreadExecutor();
    volatile String operatorTicket="",operatorError="";
    volatile boolean visible=false;
    volatile String endpoint="http://10.21.33.102:18894",key="",error="正在连接 Orin…";
    JSONObject map; volatile JSONObject state=new JSONObject(); long received=0; boolean selecting=false,follow=false;
    MapView canvas; TextView health,detail,selected,jobText,navText; Button selectButton,followButton;
    LinearLayout mapPage,manualPage; boolean manualScreen=false,manualSelected=false,manualShadow=false;
    TextView manualHealth,manualFeedback,manualMessage,manualRc; Button wakeButton;
    final Button[] manualPolicies=new Button[10]; StickView sticks;
    final Button[] navPolicies=new Button[4];
    Button presetModeButton,overrideModeButton,speedButton;
    String manualChoice="basic_normal",lastOwner=""; volatile int manualSelection=0;
    int temporaryMode=0;String temporaryEditId="";JSONObject temporaryData=new JSONObject();
    Button temporaryAddButton,temporaryReplaceButton,temporaryListButton;TextView temporaryHint;
    Spinner floor; double[] seed=null; double radius=3; int floorMode=0;
    final String rcSession=UUID.randomUUID().toString();
    final Object rcLock=new Object();
    volatile boolean rcConnected=false; boolean rcReading=false;
    long rcAt=0,rcSeq=0,rcEpoch=0,rcRequest=0,rcTicket=0,lastRcLog=0,lastRcConnect=0;
    int[] channels=new int[16]; String rcText="摇杆初始化";
    final String[] policyIds={"basic_normal","stairs_normal","platform","step_move","basic","stairs","low","high","low_v5","high_v2"};
    final String[] policyNames={"基础普通","楼梯普通","高台","踏步移动","基础 · 敏捷","楼梯 · 敏捷","低速 waypoint","高速 waypoint","低速 v5 · 45200","高速 v2 · 35400"};
    volatile JSONObject missionData=new JSONObject();
    String routeDisplaySource="";
    boolean calibrationPreview=false; Button routeViewButton;
    JSONObject calibrationData=new JSONObject(); String calibrationRoute="",calibrationPoint="";
    AlertDialog calibrationWindow; TextView calibrationLive,calibrationDetail; Spinner calibrationPicker;
    boolean calibrationLoading=false;
    JSONObject recordingData=new JSONObject(); AlertDialog recordingWindow;
    TextView recordingLive,recordingHistory; EditText recordingName,recordingPointName;
    Button recordingStart,recordingPause,recordingSave,recordingQuick,recordingModes;
    String pendingId="",message="自动导航 / 人工接管 · 分段预设：基础 / 楼梯 / 高台 / 踏步移动";

    @Override public void onCreate(Bundle b){
        super.onCreate(b);getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_FULLSCREEN);
        endpoint=getPreferences(0).getString("endpoint",endpoint);
        try(BufferedReader r=new BufferedReader(new FileReader(new File(getFilesDir(),"pairing.key")))){key=r.readLine().trim();}catch(Exception e){error="未配对，请通过 USB 安装配对文件";}
        try{map=new JSONObject(read(getAssets().open("map.json"),6000000));}catch(Exception e){throw new RuntimeException("地图资产读取失败",e);}
        loadRouteDisplay();build();initRc();io.scheduleWithFixedDelay(this::poll,0,100,TimeUnit.MILLISECONDS);rcIo.scheduleWithFixedDelay(this::sendOperator,0,50,TimeUnit.MILLISECONDS);ui.post(tick);ui.post(rcPoll);
    }
    @Override public void onResume(){super.onResume();visible=true;connectRc();}
    @Override protected void onNewIntent(android.content.Intent intent){super.onNewIntent(intent);setIntent(intent);}
    @Override public void onPause(){visible=false;invalidateRc();emergencyStop();super.onPause();}
    @Override public void onDestroy(){io.shutdownNow();rcIo.shutdownNow();commands.shutdownNow();ui.removeCallbacks(tick);ui.removeCallbacks(rcPoll);RCSDKManager.INSTANCE.disconnectRC();super.onDestroy();}
    @Override public void onBackPressed(){if(manualScreen)leaveManual();else super.onBackPressed();}
    static String read(InputStream in,int limit)throws Exception{try(InputStream s=in;ByteArrayOutputStream out=new ByteArrayOutputStream()){byte[] b=new byte[8192];int n;while((n=s.read(b))!=-1){out.write(b,0,n);if(out.size()>limit)throw new IOException("响应过大");}return out.toString("UTF-8");}}
    static String sign(String key,String text)throws Exception{Mac m=Mac.getInstance("HmacSHA256");m.init(new SecretKeySpec(key.getBytes(StandardCharsets.UTF_8),"HmacSHA256"));StringBuilder s=new StringBuilder();for(byte b:m.doFinal(text.getBytes(StandardCharsets.UTF_8)))s.append(String.format(Locale.ROOT,"%02x",b&255));return s.toString();}
    JSONObject rpc(String path,JSONObject request)throws Exception{
        String nonce=UUID.randomUUID().toString();request.put("nonce",nonce);String raw=request.toString();
        HttpURLConnection c=(HttpURLConnection)new URL(endpoint+path).openConnection();c.setInstanceFollowRedirects(false);
        c.setConnectTimeout(350);c.setReadTimeout(350);c.setRequestMethod("POST");c.setDoOutput(true);
        c.setRequestProperty("Content-Type","application/json");c.setRequestProperty("X-S10-MAC",sign(key,path+"\n"+raw));
        try{try(OutputStream out=c.getOutputStream()){out.write(raw.getBytes(StandardCharsets.UTF_8));}
            if(c.getResponseCode()!=200)throw new IOException("HTTP "+c.getResponseCode());
            String result=read(c.getInputStream(),8000000);
            if(!sign(key,path+"\n"+result).equals(c.getHeaderField("X-S10-MAC")))throw new IOException("服务端签名不匹配");
            JSONObject value=new JSONObject(result);if(!nonce.equals(value.optString("nonce")))throw new IOException("响应不属于本次请求");
            if(!value.optBoolean("ok"))throw new IOException(value.optString("error"));return value;
        }finally{c.disconnect();}
    }
    String routeCacheKey(String server){return "route_display:"+map.optString("asset_id")+":"+server;}
    void loadRouteDisplay(){
        String asset=map.optString("asset_id");
        JSONObject cached=RouteDisplayCache.restore(getPreferences(0).getString(routeCacheKey(endpoint),""),asset,endpoint);
        routeDisplaySource="上次同步路线";
        if(cached==null)try{cached=RouteDisplayCache.restore(read(getAssets().open("route-display.json"),8000000),asset,endpoint);routeDisplaySource="内置已校准路线";}catch(Exception ignored){}
        missionData=cached==null?new JSONObject():cached;
    }
    void cacheRouteDisplay(JSONObject fresh,String server){
        JSONObject mission=fresh.optJSONObject("mission");
        if(mission==null||!mission.has("catalog")||!map.optString("asset_id").equals(fresh.optString("asset_id")))return;
        try{String raw=RouteDisplayCache.snapshot(map.optString("asset_id"),server,mission).toString();
            getPreferences(0).edit().putString(routeCacheKey(server),raw).apply();
        }catch(Exception e){android.util.Log.w("GOAINavRoute","路线缓存保存失败",e);}
    }
    void poll(){if(!visible||key.length()<32)return;
        try{String server=endpoint;JSONObject request=new JSONObject();JSONObject n=state.optJSONObject("navigation");if(n!=null&&n.optBoolean("active")){request.put("navigation_run_id",n.optString("run_id"));request.put("navigation_ticket",state.optString("navigation_ticket"));}if(!missionData.has("catalog_key")||!catalogKey(state).equals(catalogKey(missionData)))request.put("include_catalog",true);JSONObject s=rpc("/state",request);
            if(!server.equals(endpoint))return;
            operatorTicket=s.getString("operator_ticket");
            if(!map.getString("asset_id").equals(s.optString("asset_id")))throw new IOException("地图版本不匹配；禁止重定位");
            cacheRouteDisplay(s,server);
            ui.post(()->{if(!server.equals(endpoint))return;state=s;JSONObject m=s.optJSONObject("mission");if(!calibrationRoute.equals(routeId())||(m!=null&&m.optInt("calibration_revision",-1)!=calibrationData.optInt("revision",-1)))calibrationPreview=false;if(m!=null&&m.has("catalog")){missionData=m;routeDisplaySource="上次同步路线";}received=SystemClock.elapsedRealtime();error="";JSONObject temporary=s.optJSONObject("navigation");if(temporary!=null&&temporary.has("temporary_waypoints"))temporaryData=temporary.optJSONObject("temporary_waypoints");canvas.invalidate();});
        }catch(Exception e){ui.post(()->{error="连接中断："+e.getMessage();received=0;});}
    }
    String catalogKey(JSONObject value){
        JSONObject m=value.optJSONObject("mission");if(m==null)m=value;
        return m.optString("catalog_key");
    }
    boolean connected(){return error.isEmpty()&&SystemClock.elapsedRealtime()-received<1800;}
    final Runnable tick=new Runnable(){public void run(){render();ui.postDelayed(this,150);}};
    TextView text(String s,int size,int color){TextView t=new TextView(this);t.setText(s);t.setTextSize(size);t.setTextColor(color);t.setPadding(dp(8),dp(4),dp(8),dp(4));return t;}
    int dp(float n){return (int)(n*getResources().getDisplayMetrics().density+.5f);}
    Button button(String s,Runnable r){Button b=new Button(this);b.setText(s);b.setTextSize(13);b.setAllCaps(false);b.setTextColor(WHITE);b.setPadding(dp(4),0,dp(4),0);b.setMinHeight(0);b.setMinimumHeight(0);GradientDrawable g=new GradientDrawable();g.setColor(CARD);g.setCornerRadius(dp(7));b.setBackground(g);b.setOnClickListener(v->r.run());return b;}
    void cell(LinearLayout row,View v,float weight){LinearLayout.LayoutParams p=new LinearLayout.LayoutParams(0,-1,weight);p.setMargins(dp(3),dp(2),dp(3),dp(2));row.addView(v,p);}
    LinearLayout row(LinearLayout parent,int h){LinearLayout r=new LinearLayout(this);parent.addView(r,new LinearLayout.LayoutParams(-1,dp(h)));return r;}
    void build(){
        FrameLayout screens=new FrameLayout(this);setContentView(screens);
        LinearLayout root=new LinearLayout(this);root.setOrientation(1);root.setBackgroundColor(BG);root.setPadding(dp(8),dp(5),dp(8),dp(5));mapPage=root;screens.addView(root,new FrameLayout.LayoutParams(-1,-1));
        LinearLayout top=row(root,38);cell(top,text("GOAI / 导航",19,WHITE),1.3f);health=text("离线地图",14,GOLD);health.setSingleLine(true);health.setEllipsize(android.text.TextUtils.TruncateAt.END);cell(top,health,3);cell(top,button("连接设置",this::settings),.8f);
        LinearLayout tools=row(root,40);routeViewButton=button("正式路线",()->{calibrationPreview=false;canvas.invalidate();});cell(tools,routeViewButton,1);cell(tools,button("全图",()->{follow=false;canvas.fit();}),1);cell(tools,button("地图原点",()->{follow=false;canvas.cx=0;canvas.cy=0;canvas.scale=dp(22);canvas.invalidate();}),1);
        followButton=button("跟随机器人",()->{follow=!follow;canvas.invalidate();});cell(tools,followButton,1);
        selectButton=button("重定位点选",()->{selecting=!selecting;temporaryMode=0;temporaryEditId="";canvas.invalidate();});cell(tools,selectButton,1);
        floor=new Spinner(this);ArrayAdapter<String> levels=new ArrayAdapter<>(this,android.R.layout.simple_spinner_dropdown_item,new String[]{"全部高度","低层 / 地面","高层 / 楼上"});floor.setAdapter(levels);floor.setOnItemSelectedListener(new android.widget.AdapterView.OnItemSelectedListener(){public void onNothingSelected(android.widget.AdapterView<?> p){}public void onItemSelected(android.widget.AdapterView<?> p,View v,int i,long id){floorMode=i;canvas.invalidate();}});cell(tools,floor,1.2f);
        LinearLayout directPolicies=row(root,36);
        presetModeButton=button("路线预设",()->chooseNavigationMode(false));cell(directPolicies,presetModeButton,1.25f);
        overrideModeButton=button("人工覆盖",()->chooseNavigationMode(true));cell(directPolicies,overrideModeButton,1.25f);
        for(int i=0;i<navPolicies.length;i++){final int k=i;navPolicies[i]=button(policyNames[i],()->{try{sendAction("override",new JSONObject().put("policy",policyIds[k]));}catch(Exception ignored){}});cell(directPolicies,navPolicies[i],1);}
        LinearLayout body=new LinearLayout(this);root.addView(body,new LinearLayout.LayoutParams(-1,0,1));canvas=new MapView();body.addView(canvas,new LinearLayout.LayoutParams(0,-1,1));
        ScrollView sideScroll=new ScrollView(this);body.addView(sideScroll,new LinearLayout.LayoutParams(dp(220),-1));
        LinearLayout side=new LinearLayout(this);side.setOrientation(1);side.setPadding(dp(6),0,0,0);sideScroll.addView(side,new ScrollView.LayoutParams(-1,-2));
        side.addView(text("临时后续点",15,TEAL));
        LinearLayout temporaryTools=row(side,36);
        temporaryAddButton=button("点选追加",()->temporaryMode(1));cell(temporaryTools,temporaryAddButton,1);
        temporaryReplaceButton=button("点选替换",()->temporaryMode(2));cell(temporaryTools,temporaryReplaceButton,1);
        LinearLayout temporaryActions=row(side,34);
        temporaryListButton=button("后续点 0",this::temporaryDialog);cell(temporaryActions,temporaryListButton,1.2f);
        cell(temporaryActions,button("清空",()->temporaryAction("clear",new JSONObject())),.8f);
        temporaryHint=text("按点选顺序经过，再续接原路线。",11,GOLD);side.addView(temporaryHint);
        side.addView(text("附近重定位",15,TEAL));selected=text("开启点选后，点击地图上机器人所在位置附近。",12,WHITE);side.addView(selected,new LinearLayout.LayoutParams(-1,dp(42)));
        LinearLayout radii=row(side,28);for(int n:new int[]{1,2,3,5}){final int v=n;cell(radii,button(n+" m",()->{radius=v;canvas.invalidate();}),1);}
        LinearLayout adjust=row(side,28);cell(adjust,button("高度 / 坐标",this::coordinateDialog),1);cell(adjust,button("从定位取点",()->{JSONArray p=state.optJSONArray("pose");if(p!=null){seed=new double[]{p.optDouble(0),p.optDouble(1),p.optDouble(2)};canvas.invalidate();}}),1);
        LinearLayout act=row(side,38);Button send=button("在此附近重定位",this::confirmSeed);send.setTextColor(TEAL);cell(act,send,1);
        LinearLayout cancel=row(side,28);cell(cancel,button("取消本次搜索",this::cancel),1);
        jobText=text("尚未提交定位请求",11,GOLD);side.addView(jobText,new LinearLayout.LayoutParams(-1,-2));
        LinearLayout manual=row(root,40);
        cell(manual,button("人工接管",()->enterManual(true)),1);
        cell(manual,button("启动 / 恢复",()->sendAction("start",new JSONObject())),1);
        cell(manual,button("策略 / 分段",this::policyDialog),1);
        cell(manual,button("重新规划",this::replanDialog),1);
        cell(manual,button("设置进度",this::progressDialog),1);
        speedButton=button("巡航速度",this::speedDialog);cell(manual,speedButton,1);
        cell(manual,button("停止",this::emergencyStop),.8f);
        cell(manual,button("阻尼",()->new AlertDialog.Builder(this).setTitle("阻尼 · 卸力").setMessage("立即取消导航和人工输入，请确认机器人可卸力。").setNegativeButton("取消",null).setPositiveButton("发送阻尼",(d,w)->sendAction("damping",new JSONObject())).show()),.8f);
        detail=text("地图加载完成；等待连接。",12,WHITE);root.addView(detail);
        LinearLayout nav=row(root,58);navText=text("导航未启动",11,TEAL);cell(nav,navText,3.4f);cell(nav,button("路线录制",this::recordingDialog),1);cell(nav,button("点位校准",this::calibrationDialog),1);cell(nav,button("导航操作",this::navigationDialog),1);Button stop=button("暂停导航",this::emergencyStop);stop.setTextColor(GOLD);cell(nav,stop,.9f);
        buildManual(screens);
    }
    void chooseNavigationMode(boolean override){
        JSONObject nav=state.optJSONObject("navigation");String policy=nav==null?"basic":nav.optString("policy","basic");
        if(policy.equals("low")||policy.equals("high")||policy.equals("low_v5")||policy.equals("high_v2"))policy="basic_normal";
        try{sendAction("override",new JSONObject().put("policy",override?policy:JSONObject.NULL));}catch(Exception ignored){}
    }
    void sendOperator(){
        String ticket=operatorTicket;if(!visible||key.length()<32||ticket.isEmpty())return;
        try{JSONObject body=new JSONObject().put("ticket",ticket).put("session",rcSession);
            synchronized(rcLock){long age=SystemClock.elapsedRealtime()-rcAt;
                body.put("fresh",visible&&rcConnected&&rcAt!=0&&age>=0&&age<=200).put("sample_seq",rcSeq).put("sample_age_ms",age);
                body.put("axes",new JSONArray(new double[]{norm(channels[2]),-norm(channels[3]),-norm(channels[0])}));body.put("stop",channels[9]>=1900);
            }
            JSONObject reply=rpc("/operator",body);operatorTicket=reply.getString("operator_ticket");operatorError="";
        }catch(Exception e){if(operatorTicket.equals(ticket))operatorTicket="";operatorError=e.getMessage();}
    }
    void buildManual(FrameLayout screens){
        manualPage=new LinearLayout(this);manualPage.setOrientation(1);manualPage.setBackgroundColor(BG);manualPage.setPadding(dp(8),dp(5),dp(8),dp(5));screens.addView(manualPage,new FrameLayout.LayoutParams(-1,-1));manualPage.setVisibility(View.GONE);
        LinearLayout top=row(manualPage,42);cell(top,text("GOAI / 人工控制",19,WHITE),1.5f);manualHealth=text("等待连接",13,GOLD);cell(top,manualHealth,1.7f);cell(top,button("路线录制",this::recordingDialog),1);recordingQuick=button("标记路点",()->recordingAction("mark",new JSONObject()));cell(top,recordingQuick,1);cell(top,button("点位校准",this::calibrationDialog),1);cell(top,button("返回地图",this::leaveManual),1);
        manualPage.addView(text("摇杆自动启用 · 选择策略即可控制，起立后自动接收摇杆",13,TEAL));
        LinearLayout body=new LinearLayout(this);manualPage.addView(body,new LinearLayout.LayoutParams(-1,0,1));
        LinearLayout policies=new LinearLayout(this);policies.setOrientation(1);body.addView(policies,new LinearLayout.LayoutParams(0,-1,1.5f));
        policies.addView(text("运控策略 · 人工可选十种",14,WHITE));
        LinearLayout official=row(policies,50),normal=row(policies,50),waypoint=row(policies,50),updated=row(policies,50);
        for(int i=0;i<policyIds.length;i++){final int k=i;manualPolicies[i]=button(policyNames[i],()->chooseManualPolicy(k));cell(i<4?official:i<6?normal:i<8?waypoint:updated,manualPolicies[i],1);}
        manualFeedback=text("请先选择运控策略",13,GOLD);ScrollView feedbackScroll=new ScrollView(this);feedbackScroll.addView(manualFeedback);policies.addView(feedbackScroll,new LinearLayout.LayoutParams(-1,0,1));
        LinearLayout input=new LinearLayout(this);input.setOrientation(1);input.setPadding(dp(12),0,0,0);body.addView(input,new LinearLayout.LayoutParams(0,-1,1));
        input.addView(text("G12 实体摇杆 · 实时输入",14,WHITE));sticks=new StickView();input.addView(sticks,new LinearLayout.LayoutParams(-1,0,1));
        manualRc=text("等待摇杆",12,WHITE);input.addView(manualRc);input.addView(text("A 接管  ·  B / D 停止  ·  C 起立",11,GOLD));
        LinearLayout actions=row(manualPage,46);
        wakeButton=button("唤醒",()->sendAction("wake",new JSONObject()));cell(actions,wakeButton,.85f);
        cell(actions,button("起立",()->sendAction("stand",new JSONObject())),1);cell(actions,button("趴下",()->sendAction("lie",new JSONObject())),1);
        Button stop=button("停止 / 锁定",this::emergencyStop);stop.setTextColor(GOLD);cell(actions,stop,1.2f);
        cell(actions,button("阻尼 / 卸力",()->new AlertDialog.Builder(this).setTitle("阻尼 · 卸力").setMessage("立即取消人工输入，请确认机器人可卸力。").setNegativeButton("取消",null).setPositiveButton("发送阻尼",(d,w)->sendAction("damping",new JSONObject())).show()),1.2f);
        manualMessage=text("",12,WHITE);manualMessage.setMaxLines(2);manualPage.addView(manualMessage,new LinearLayout.LayoutParams(-1,dp(48)));
    }
    void enterManual(boolean pause){
        enterManual(pause,false);
    }
    void enterManual(boolean pause,boolean shadow){
        if(manualScreen)return;
        manualScreen=true;manualSelected=true;manualShadow=shadow;manualSelection++;
        JSONObject nav=state.optJSONObject("navigation");manualChoice=nav==null?"basic":nav.optString("policy","basic");
        mapPage.setVisibility(View.GONE);manualPage.setVisibility(View.VISIBLE);
        if(pause){startManual(manualShadow);message="已进入人工控制，摇杆自动启用";}
        else {manualSelected=true;message="已由摇杆接管，可在此切换策略";}
        renderManual();
    }
    void leaveManual(){
        manualSelection++;manualSelected=false;emergencyStop();sendAction("pause",new JSONObject());
        manualScreen=false;lastOwner="manual";manualPage.setVisibility(View.GONE);mapPage.setVisibility(View.VISIBLE);canvas.invalidate();
    }
    void chooseManualPolicy(int index){
        manualChoice=policyIds[index];manualSelected=true;
        message="正在选择 "+policyNames[index];startManual(manualShadow);
        renderManual();
    }
    void startManual(boolean shadow){
        JSONObject control=state.optJSONObject("control");boolean monitor=control!=null&&control.optString("backend").equals("monitor");
        ++manualSelection;String action=shadow||monitor?"manual_shadow":"manual";
        try{sendAction(action,new JSONObject().put("policy",manualChoice).put("operator_session",rcSession));}catch(Exception ignored){}
    }
    String policyName(String id){for(int i=0;i<policyIds.length;i++)if(policyIds[i].equals(id))return policyNames[i];return "未确认";}
    String controlReason(JSONObject control){if(control==null)return "等待控制网关";String r=control.optString("health_reason");switch(r){case "SLEEP_ACTIVE":return "机器人休眠中，请先唤醒";case "HARD_STOP_ACTIVE":return "机器人硬急停已触发";case "CHARGING":return "机器人正在充电";case "MOTION_TELEMETRY_STALE":case "BASIC_STATUS_STALE":return "机器人反馈过期，保持停止";case "ZERO_GUARDIAN_NOT_READY_OR_LATCHED":return "停止保护未就绪，保持锁定";case "":case "null":return control.optString("detail",control.optString("reason"));default:return "控制未就绪："+r;}}
    String navigationReason(String value){switch(value){case "JOINING_ROUTE":return "正在接入最近前方目标";case "HOLD_OBSTACLE":return "障碍物占用，等待通行";case "HOLD_LOCALIZATION":return "等待有效定位";case "HOLD_FORWARD_PERCEPTION":return "等待前方雷达";case "HOLD_OFF_ROUTE":return "已偏离路线，请重新规划";case "HOLD_ROUTE_VALIDATION":return "此路段尚未通过通行验证";case "POLICY_SWITCH_PAUSE":return "原地暂停切换策略，完成后自动继续";case "WAIT_POLICY_FEEDBACK":return "等待策略及执行反馈";case "FOLLOWING":return "沿路线导航中";case "COMPLETE":return "路线完成";case "CONFIRM_WAYPOINT":return "确认到达路线点";case "RECOVERING":return "等待定位稳定";default:return "暂停后需显式恢复导航";}}
    void renderManual(){if(!manualScreen)return;
        boolean link=connected();JSONObject nav=state.optJSONObject("navigation"),control=state.optJSONObject("control");
        boolean active=link&&nav!=null&&nav.optString("owner").equals("manual");
        boolean monitor=control!=null&&control.optString("backend").equals("monitor");
        String actual=link&&control!=null?control.optString("confirmed"):"none";
        manualHealth.setText(!link?"Orin 连接中断":monitor?"监控模式 · 不驱动实机":control!=null&&control.optBoolean("sleeping")?"实机已连接 · 休眠中":active?"实机 · 人工控制中":nav!=null&&nav.optString("owner").equals("auto")?"实机 · 正在暂停导航":"实机已连接 · 待命");manualHealth.setTextColor(active?TEAL:GOLD);
        for(int i=0;i<policyIds.length;i++){Button b=manualPolicies[i];boolean chosen=manualChoice.equals(policyIds[i]);
            b.setText((chosen?"● ":"")+policyNames[i]);b.setTextColor(chosen?TEAL:WHITE);
            GradientDrawable g=(GradientDrawable)b.getBackground();g.setStroke(dp(chosen?2:0),TEAL);
        }
        String selectedPolicy="已选择："+policyName(manualChoice);
        String feedback=monitor?"实机反馈：监控模式，不切换实机":"实机反馈："+policyName(actual);
        boolean enabled=active&&control!=null&&control.optBoolean("enabled");
        boolean waitingInput=active&&nav.optString("state").equals("MANUAL_WAIT_INPUT");
        String phase=active?(!nav.optBoolean("execution_requested")?"影子试控中 · 不驱动实机":waitingInput?"自动控制已接通，等待实体摇杆数据":enabled?"摇杆自动控制中":"等待所选策略；起立后自动控制"):"控制已停止；选择策略或起立即可恢复";
        manualFeedback.setText(selectedPolicy+"\n"+feedback+"\n"+(link?phase:"连接中断，等待重新连接")+(link&&!monitor?"\n"+controlReason(control):""));
        wakeButton.setEnabled(link&&!monitor&&control!=null&&control.optBoolean("sleeping"));wakeButton.setAlpha(wakeButton.isEnabled()?1f:.45f);
        long age;double x,y,w;boolean fresh;synchronized(rcLock){age=rcAt==0?-1:SystemClock.elapsedRealtime()-rcAt;fresh=rcConnected&&age>=0&&age<=200;x=norm(channels[2]);y=-norm(channels[3]);w=-norm(channels[0]);}
        manualRc.setText(String.format(Locale.ROOT,"%s\n前后 %+.2f  横移 %+.2f  转向 %+.2f",fresh?"摇杆在线 · "+age+" ms":"摇杆未就绪",x,y,w));manualRc.setTextColor(fresh?WHITE:GOLD);sticks.invalidate();
        manualMessage.setText(message+"\n返回地图后保持暂停；需在地图页显式恢复导航。");
    }
    final class StickView extends View {
        final Paint p=new Paint(Paint.ANTI_ALIAS_FLAG);StickView(){super(MainActivity.this);setContentDescription("实体摇杆实时位置，仅显示");}
        @Override protected void onDraw(Canvas c){super.onDraw(c);float r=Math.max(dp(8),Math.min(getWidth()/5f,(getHeight()-dp(30))/2f)),cy=r+dp(3);
            double x,y,w;boolean fresh;synchronized(rcLock){x=norm(channels[2]);y=-norm(channels[3]);w=-norm(channels[0]);fresh=rcConnected&&rcAt!=0&&SystemClock.elapsedRealtime()-rcAt<=200;}
            for(int i=0;i<2;i++){float cx=getWidth()*(i==0?.25f:.75f);p.setStyle(Paint.Style.FILL);p.setColor(CARD);c.drawCircle(cx,cy,r,p);p.setColor(Color.rgb(72,97,113));p.setStrokeWidth(dp(1));c.drawLine(cx-r,cy,cx+r,cy,p);c.drawLine(cx,cy-r,cx,cy+r,p);p.setColor(fresh?TEAL:GOLD);c.drawCircle(cx+(float)(fresh?(i==0?-y:-w):0)*r*.75f,cy-(float)(fresh&&i==0?x:0)*r*.75f,dp(6),p);p.setColor(WHITE);p.setTextSize(dp(12));p.setTextAlign(Paint.Align.CENTER);c.drawText(i==0?"前后 / 横移":"转向",cx,cy+r+dp(20),p);}
        }
    }
    String stateName(String s){switch(s){case "TRACKING":return "定位正常";case "DEGRADED":return "降级可用";case "PREDICT_ONLY":return "短时推算";case "ODOM_BRIDGE":return "里程计定位";case "LOST":return "定位失效";case "RELOCALIZING":return "重定位中";case "WAIT_SEED":return "等待点选初始化";default:return s;}}
    String jobName(String s){switch(s){case "QUEUED":return "请求已提交";case "SEARCHING":return "正在几何匹配";case "SUCCEEDED":return "重定位成功";case "FAILED":return "重定位未通过";case "CANCELLED":return "搜索已取消";default:return "未提交";}}
    void render(){if(canvas==null)return;renderCalibrationLive();renderRecording();
        if(SystemClock.elapsedRealtime()-lastRcLog>5000){lastRcLog=SystemClock.elapsedRealtime();android.util.Log.i("GOAINavRC","seq="+rcSeq+" age_ms="+(rcAt==0?-1:SystemClock.elapsedRealtime()-rcAt));}
        JSONObject currentNav=state.optJSONObject("navigation");if(connected()&&currentNav!=null){String owner=currentNav.optString("owner");if(owner.equals("manual")&&!lastOwner.equals("manual")&&!manualScreen)enterManual(false);lastOwner=owner;}
        if(manualScreen){renderManual();return;}
        boolean link=connected(),valid=link&&state.optBoolean("valid");JSONObject sol=state.optJSONObject("solution");double age=sol==null?Double.NaN:sol.optDouble("odom_age_s");
        health.setText(link?stateName(state.optString("mode"))+"  |  里程计 "+(Double.isNaN(age)?"—":String.format(Locale.ROOT,"%.2fs",age)):error.isEmpty()?"连接过期 / 保留最后位置":error);
        health.setTextColor(valid?TEAL:GOLD);selectButton.setText(selecting?"● 重定位点选":"重定位点选");followButton.setText(follow?"正在跟随":"跟随机器人");
        JSONArray temporaryPoints=temporaryData.optJSONArray("pending");int temporaryCount=temporaryPoints==null?0:temporaryPoints.length();
        temporaryListButton.setText("后续点 "+temporaryCount);
        temporaryAddButton.setText((temporaryMode==1?"● ":"")+"点选追加");temporaryReplaceButton.setText((temporaryMode==2?"● ":"")+"点选替换");
        temporaryHint.setText(temporaryMode==3?"点击地图，修改所选临时点。":temporaryMode==2?"点击地图，替换全部未经过的临时点；立即生效。":temporaryMode==1?"点击地图即追加；拖动、双指缩放不加点。":"临时点按顺序经过，再续接原路线。可在队列中改点或删点。");
        selected.setText(seed==null?"开启点选后，点击机器人当前位置附近。":String.format(Locale.ROOT,"X %.2f   Y %.2f\nZ %.2f m  · 半径 %.0f m\n高度范围 ±0.8 m",seed[0],seed[1],seed[2],radius));
        JSONObject j=state.optJSONObject("job");String job=message;if(j!=null&&!j.optString("state").equals("IDLE")){job+="\n"+jobName(j.optString("state"))+"\n"+j.optString("reason");if(j.optString("state").equals("SEARCHING"))job+="\n"+j.optString("last_rejection");}jobText.setText(job);
        JSONArray p=state.optJSONArray("pose");String position=p==null?"尚无通过验证的位置":String.format(Locale.ROOT,"X %.2f  Y %.2f  Z %.2f  朝向 %.1f°",p.optDouble(0),p.optDouble(1),p.optDouble(2),Math.toDegrees(p.optDouble(3)));
        JSONObject sensors=state.optJSONObject("sensors");String inputs="";if(sensors!=null)for(String name:new String[]{"front","rear"}){double t=sensors.optDouble("/wym/slam/"+name+"/points",999);inputs+="  "+(name.equals("front")?"前雷达 ":"后雷达 ")+(t>10?"缺失":String.format(Locale.ROOT,"%.1fs",t));}
        detail.setText(position+inputs+"  · "+rcText);canvas.invalidate();
        JSONObject nav=state.optJSONObject("navigation"),control=state.optJSONObject("control");
        if(nav!=null){speedButton.setText(String.format(Locale.ROOT,"巡航 %.1f m/s",nav.optDouble("cruise_mps",2.0)));String backend=control==null?"未接入":control.optString("backend");
            String choice=nav.optString("policy","basic");
            boolean preset=nav.optString("policy_mode",nav.isNull("override")?"route_preset":"override").equals("route_preset");
            for(int i=0;i<2;i++){Button b=i==0?presetModeButton:overrideModeButton;boolean chosen=i==0?preset:!preset;
                b.setText((chosen?"● ":"")+(i==0?"路线预设":"人工覆盖"));b.setTextColor(chosen?TEAL:WHITE);
                ((GradientDrawable)b.getBackground()).setStroke(dp(chosen?2:0),TEAL);
            }
            for(int i=0;i<navPolicies.length;i++){Button b=navPolicies[i];boolean chosen=choice.equals(policyIds[i]);
                boolean confirmed=control!=null&&choice.equals(control.optString("actual_policy",control.optString("confirmed")));int color=confirmed?TEAL:GOLD;
                b.setText((chosen?"● ":"")+policyNames[i]);b.setTextColor(chosen?color:WHITE);
                ((GradientDrawable)b.getBackground()).setStroke(dp(chosen?2:0),color);
            }
            String owner=nav.optString("owner");owner=owner.equals("auto")?"自动":owner.equals("manual")?"人工":"暂停";
            String actual=control==null?"none":control.optString("actual_policy",control.optString("confirmed"));for(int i=0;i<policyIds.length;i++)if(actual.equals(policyIds[i]))actual=policyNames[i];if(actual.equals("none"))actual="未确认";
            String status=control!=null&&(!control.optBoolean("healthy")||control.optString("phase").equals("FAILED"))?controlReason(control):navigationReason(nav.optString("state"));
            if(control!=null&&control.optBoolean("policy_switch_paused"))status=control.optString("detail","原地暂停切换策略");
            if(nav.optString("state").equals("PROGRESS_SET"))status="进度已设置；点击启动 / 恢复导航继续";
            if(nav.optString("state").equals("JOINING_ROUTE")&&!nav.isNull("manual_target"))status="正在接入手动指定目标";
            JSONObject gaitReply=control==null?null:control.optJSONObject("gait_response");
            if(gaitReply!=null&&gaitReply.optInt("error_code")!=0&&control.optString("phase").equals("FAILED")){
                int code=gaitReply.optInt("error_code");status+="；最近底盘步态回复 "+String.format(Locale.ROOT,"0x%04X",code)+(code==0xE008?" 不允许的操作":"");
            }
            if(control!=null&&control.optString("phase").equals("FAILED"))jobText.append("\n步态切换反馈\n"+status);
            JSONObject temporaryNext=nav.optJSONObject("next_temporary");if(temporaryNext!=null){JSONObject returning=nav.optJSONObject("temporary_return");status="先往 "+temporaryNext.optString("name")+(returning==null?"":" → "+returning.optString("name"))+" · "+status;}
            navText.setText(routeName(nav.optString("route_id"))+" · "+owner+" · "+(preset?"路线预设":"人工覆盖")+" · "+nav.optString("policy_name")+" → "+nav.optString("target_name")+
                String.format(Locale.ROOT,"\nv %.2f  ω %.2f · 实际 %s · %s",nav.optDouble("vx"),nav.optDouble("wz"),actual,backend.equals("monitor")?"监控模式":backend.equals("hardware")?"实机控制":backend)+"\n"+status);
        }
        if(!link){JSONObject catalog=missionData.optJSONObject("catalog"),route=catalog==null?null:catalog.optJSONObject(routeId());if(route!=null)navText.setText(routeName(routeId())+" · "+route.optJSONArray("waypoints").length()+" 个路点\n离线显示 · "+routeDisplaySource+"\n连接后自动同步当前路线");}
        if(control!=null){JSONObject action=control.optJSONObject("last_action");if(action!=null){String id=action.optString("request_id");if(id.length()>8)id=id.substring(0,8);jobText.append("\n网关 "+action.optString("kind")+" ["+id+"]\n"+action.optString("state")+" "+action.optString("reason"));}}
    }
    void navigationDialog(){
        String[] options={"选择路线 / 从头重开","录制新路线","影子导航（不行走）","实机导航 / 显式恢复","人工接管","影子人工控制","起立","趴下","停止并锁定"};
        new AlertDialog.Builder(this).setTitle("导航与人工控制").setItems(options,(d,i)->{
            if(i==0)routePicker();else if(i==1)recordingDialog();
            else if(i==4||i==5){enterManual(true,i==5);}
            else sendAction(new String[]{"shadow","start","manual","manual_shadow","stand","lie","pause"}[i-2],new JSONObject());
        }).show();
    }
    String routeName(String id){JSONObject catalog=missionData.optJSONObject("catalog"),route=catalog==null?null:catalog.optJSONObject(id);return route==null?(id.equals("full")?"室外":id.equals("indoor")?"室内":id):route.optString("display_name",id);}
    void routePicker(){withMission(m->{JSONObject catalog=m.optJSONObject("catalog");if(catalog==null)return;ArrayList<String> ids=new ArrayList<>(),labels=new ArrayList<>();
        for(Iterator<String> it=catalog.keys();it.hasNext();){String id=it.next();JSONObject r=catalog.optJSONObject(id);ids.add(id);labels.add((id.equals(routeId())?"● ":"")+routeName(id)+" · "+r.optJSONArray("waypoints").length()+" 点");}
        new AlertDialog.Builder(this).setTitle("选择已保存路线 · 从起点重新开始").setItems(labels.toArray(new String[0]),(d,i)->navAction("select",ids.get(i))).setNegativeButton("取消",null).show();
    });}
    void recordingDialog(){
        ScrollView scroll=new ScrollView(this);LinearLayout body=new LinearLayout(this);body.setOrientation(1);body.setBackgroundColor(BG);body.setPadding(dp(8),dp(4),dp(8),dp(4));scroll.addView(body);
        recordingLive=text("正在读取录制状态…",12,TEAL);body.addView(recordingLive);
        body.addView(text("人工带走：开始时记起点，手动标记路点，保存时记终点。连续轨迹和模式切换位置自动记录，旧路线保留。",12,WHITE));
        recordingName=new EditText(this);recordingName.setSingleLine(true);recordingName.setTextColor(WHITE);recordingName.setTextSize(13);recordingName.setHintTextColor(GOLD);recordingName.setHint("新路线名称");
        recordingName.setText("路线 "+new java.text.SimpleDateFormat("MM-dd HH:mm",Locale.ROOT).format(new Date()));body.addView(recordingName,new LinearLayout.LayoutParams(-1,dp(40)));
        LinearLayout actions=row(body,40);recordingStart=button("开始录制 · 记起点",()->{try{recordingAction("start",new JSONObject().put("name",recordingName.getText().toString()));}catch(Exception ignored){}});cell(actions,recordingStart,1.4f);
        recordingPause=button("暂停录制",()->recordingAction(recordingData.optString("status").equals("paused")?"resume":"pause",new JSONObject()));cell(actions,recordingPause,1);
        recordingSave=button("保存新路线 · 记终点",()->recordingAction("save",new JSONObject()));recordingSave.setTextColor(TEAL);cell(actions,recordingSave,1.5f);
        LinearLayout mark=row(body,40);recordingPointName=new EditText(this);recordingPointName.setSingleLine(true);recordingPointName.setTextColor(WHITE);recordingPointName.setTextSize(13);recordingPointName.setHintTextColor(GOLD);recordingPointName.setHint("路点名（可留空自动编号）");cell(mark,recordingPointName,2);
        cell(mark,button("标记当前路点",()->{try{recordingAction("mark",new JSONObject().put("name",recordingPointName.getText().toString()));}catch(Exception ignored){}}),1.3f);cell(mark,button("撤销末个标记",()->recordingAction("undo",new JSONObject())),1.1f);
        LinearLayout modes=row(body,38);recordingModes=button("选择运控模式",()->new AlertDialog.Builder(this).setTitle("人工运控 · 切换位置自动记录").setItems(policyNames,(d,i)->{if(!manualScreen)enterManual(false);chooseManualPolicy(i);}).show());cell(modes,recordingModes,1.8f);
        cell(modes,button("返回人工控制",()->{recordingWindow.dismiss();enterManual(true);}),1.2f);cell(modes,button("暂停并看轨迹",()->{refreshRecording();recordingWindow.dismiss();if(manualScreen)leaveManual();else emergencyStop();follow=true;canvas.invalidate();}),1.2f);cell(modes,button("已保存路线",this::routePicker),1.1f);
        recordingHistory=text("",12,WHITE);body.addView(recordingHistory);
        recordingWindow=new AlertDialog.Builder(this).setTitle("新路线录制 · 手动标记路点").setView(scroll).setNegativeButton("关闭面板（继续录制）",null).create();
        recordingWindow.setOnDismissListener(d->{recordingLive=null;recordingHistory=null;recordingName=null;recordingPointName=null;recordingStart=null;recordingPause=null;recordingSave=null;recordingModes=null;});
        recordingWindow.show();recordingWindow.getWindow().setLayout((int)(getResources().getDisplayMetrics().widthPixels*.94),-2);refreshRecording();
    }
    void refreshRecording(){io.execute(()->{try{JSONObject reply=rpc("/state",new JSONObject().put("recording_preview",true));ui.post(()->{recordingData=reply.optJSONObject("route_recording");if(recordingData==null)recordingData=new JSONObject();if(recordingName!=null&&!recordingData.optString("name").isEmpty()&&!recordingData.optString("status").equals("saved"))recordingName.setText(recordingData.optString("name"));renderRecording();canvas.invalidate();});}catch(Exception e){ui.post(()->Toast.makeText(this,e.getMessage(),Toast.LENGTH_LONG).show());}});}
    void recordingAction(String action,JSONObject extra){sendAction("recording_"+action,extra,reply->{recordingData=reply.optJSONObject("route_recording");if(recordingData==null)recordingData=new JSONObject();if(action.equals("mark")&&recordingPointName!=null)recordingPointName.setText("");
        message=action.equals("save")?"新路线已保存："+reply.optString("route_name")+"；在导航操作中选择路线即可":"录制已更新";Toast.makeText(this,message,Toast.LENGTH_SHORT).show();if(action.equals("save"))withMission(m->canvas.invalidate());renderRecording();
    },failure->{Toast.makeText(this,failure,Toast.LENGTH_LONG).show();if(recordingLive!=null)recordingLive.setText(failure);});}
    void renderRecording(){
        JSONObject mission=state.optJSONObject("mission"),latest=mission==null?null:mission.optJSONObject("route_recording");
        if(latest!=null)try{for(Iterator<String> it=latest.keys();it.hasNext();){String k=it.next();recordingData.put(k,latest.get(k));}}catch(Exception ignored){}
        String status=recordingData.optString("status","idle");boolean active=status.equals("recording"),paused=status.equals("paused");
        if(recordingQuick!=null){recordingQuick.setEnabled(active);recordingQuick.setText(active?"标记路点 · "+recordingData.optInt("mark_count"):"标记路点");}
        if(recordingLive==null)return;
        JSONObject sample=recordingData.optJSONObject("latest");String policy=mission==null?manualChoice:mission.optString("manual_policy",manualChoice);JSONObject control=state.optJSONObject("control");
        String actual=control==null?"—":control.optString("actual_policy",control.optString("confirmed","—"));
        recordingLive.setText((active?"● 正在录制":paused?"录制已暂停":status.equals("saved")?"已保存":"尚未开始")+" · "+recordingData.optString("name")+"\n轨迹 "+recordingData.optInt("sample_count")+" 帧 · 标记 "+recordingData.optInt("mark_count")+" 个 · 模式切换 "+recordingData.optInt("switch_count")+" 次\n"+(sample==null?"等待起点":calibrationXYZ(sample.optJSONArray("xyz")))+" · 实际 "+policyName(actual)+(recordingData.optString("error").isEmpty()?"":"\n"+recordingData.optString("error")));
        recordingName.setEnabled(!active&&!paused);recordingStart.setEnabled(!active&&!paused);recordingPause.setEnabled(active||paused);recordingPause.setText(paused?"继续录制":"暂停录制");recordingSave.setEnabled(active||paused);recordingModes.setText("选择运控 · "+policyName(policy));
        JSONArray marks=recordingData.optJSONArray("marks");String history="";if(marks!=null)for(int i=Math.max(0,marks.length()-4);i<marks.length();i++){JSONObject p=marks.optJSONObject(i);history+=p.optString("name")+" · "+policyName(p.optString("policy"))+" · "+calibrationXYZ(p.optJSONArray("xyz"))+"\n";}
        if(recordingData.optInt("gap_count")>0)history+="定位中断 / 重定位记录 "+recordingData.optInt("gap_count")+" 次，可在地图预览和日志中核对。";
        recordingHistory.setText(history);
    }
    void calibrationDialog(){
        calibrationRoute=routeId();
        ScrollView scroll=new ScrollView(this);LinearLayout body=new LinearLayout(this);body.setOrientation(1);body.setBackgroundColor(BG);body.setPadding(dp(8),dp(5),dp(8),dp(5));scroll.addView(body);
        calibrationLive=text("读取当前位置…",12,TEAL);body.addView(calibrationLive);
        body.addView(text("先重定位对齐地图；人工走到实际通行位置后记录。草稿编辑时摇杆可继续控制。",11,WHITE));
        calibrationPicker=new Spinner(this);body.addView(calibrationPicker,new LinearLayout.LayoutParams(-1,dp(38)));
        calibrationPicker.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener(){
            public void onNothingSelected(AdapterView<?> p){}
            public void onItemSelected(AdapterView<?> p,View v,int i,long id){JSONArray items=calibrationData.optJSONArray("items");if(!calibrationLoading&&items!=null&&i<items.length()){calibrationPoint=items.optJSONObject(i).optString("id");renderCalibrationSelection();}}
        });
        calibrationDetail=text("正在读取校准草稿…",12,GOLD);body.addView(calibrationDetail);
        LinearLayout first=row(body,38);cell(first,button("记录当前位置",()->calibrationAction("record",new JSONObject())),1);cell(first,button("编辑 XYZ",this::calibrationCoordinates),1);cell(first,button("后插校正点",()->calibrationAction("insert",new JSONObject())),1);
        LinearLayout offsets=row(body,36);cell(offsets,button("向左 10 cm",()->calibrationOffset("left",.1)),1);cell(offsets,button("向右 10 cm",()->calibrationOffset("left",-.1)),1);cell(offsets,button("向前 10 cm",()->calibrationOffset("forward",.1)),1);cell(offsets,button("向后 10 cm",()->calibrationOffset("forward",-.1)),1);
        body.addView(text("前后左右以该点处路线方向为准；插入点连续经过，沿用所在段步态。",11,WHITE));
        LinearLayout edits=row(body,36);cell(edits,button("复原 / 删除此点",()->calibrationAction("reset_point",new JSONObject())),1.2f);cell(edits,button("撤销一步",()->calibrationAction("undo",new JSONObject())),1);cell(edits,button("查看草稿地图",()->{calibrationPreview=true;calibrationWindow.dismiss();canvas.invalidate();}),1);
        LinearLayout history=row(body,36);cell(history,button("放弃草稿",()->calibrationAction("discard",new JSONObject())),1);cell(history,button("恢复原路线到草稿",()->calibrationAction("reset_route",new JSONObject())),1.4f);cell(history,button("上版校准到草稿",()->calibrationAction("previous",new JSONObject())),1.3f);
        LinearLayout apply=row(body,40);Button save=button("暂停控制并应用校准",()->calibrationAction("apply",new JSONObject()));save.setTextColor(TEAL);cell(apply,save,2);cell(apply,button("刷新",this::refreshCalibration),1);
        calibrationWindow=new AlertDialog.Builder(this).setTitle("路线点位校准 · 草稿自动保存").setView(scroll).setNegativeButton("关闭",null).create();
        calibrationWindow.setOnDismissListener(d->{calibrationLive=null;calibrationDetail=null;calibrationPicker=null;});
        calibrationWindow.show();calibrationWindow.getWindow().setLayout((int)(getResources().getDisplayMetrics().widthPixels*.94),-2);
        refreshCalibration();
    }
    void refreshCalibration(){
        String requestedRoute=calibrationRoute;
        io.execute(()->{try{JSONObject reply=rpc("/state",new JSONObject().put("calibration_route",requestedRoute));ui.post(()->{if(requestedRoute.equals(calibrationRoute))showCalibration(reply.optJSONObject("calibration"),calibrationPoint);});}
            catch(Exception e){ui.post(()->{message=e.getMessage();if(calibrationDetail!=null)calibrationDetail.setText(message);});}});
    }
    JSONObject calibrationItem(){JSONArray items=calibrationData.optJSONArray("items");if(items!=null)for(int i=0;i<items.length();i++){JSONObject item=items.optJSONObject(i);if(item.optString("id").equals(calibrationPoint))return item;}return null;}
    void showCalibration(JSONObject data,String chosen){
        if(data==null)return;calibrationData=data;JSONArray items=data.optJSONArray("items");if(items==null)return;
        int selection=-1;String[] labels=new String[items.length()];
        for(int i=0;i<items.length();i++){JSONObject item=items.optJSONObject(i);labels[i]=item.optString("name")+(item.optString("kind").equals("control")?" · 插入点":item.optBoolean("modified")?" · 已校准":"");if(item.optString("id").equals(chosen))selection=i;}
        if(selection<0){JSONObject nav=state.optJSONObject("navigation");String name=nav==null?"":nav.optString("target_name");selection=0;for(int i=0;i<items.length();i++)if(items.optJSONObject(i).optString("name").equals(name)){selection=i;break;}}
        calibrationPoint=items.optJSONObject(selection).optString("id");
        if(calibrationPicker!=null){calibrationLoading=true;ArrayAdapter<String> adapter=new ArrayAdapter<>(this,android.R.layout.simple_spinner_dropdown_item,labels);calibrationPicker.setAdapter(adapter);calibrationPicker.setSelection(selection);calibrationLoading=false;}
        renderCalibrationSelection();canvas.invalidate();
    }
    String calibrationXYZ(JSONArray p){return p==null?"—":String.format(Locale.ROOT,"X %.3f  Y %.3f  Z %.3f",p.optDouble(0),p.optDouble(1),p.optDouble(2));}
    void renderCalibrationLive(){
        if(calibrationLive==null)return;JSONObject sol=state.optJSONObject("solution");JSONArray pose=sol==null?null:sol.optJSONArray("measurement_pose");if(pose==null&&sol!=null)pose=sol.optJSONArray("pose");
        double age=sol==null?Double.NaN:sol.optDouble("odom_age_s");
        calibrationLive.setText((connected()?stateName(state.optString("mode")):"连接中断")+" · 实测 "+calibrationXYZ(pose)+(Double.isNaN(age)?"":String.format(Locale.ROOT," · %.2f s",age)));
    }
    void renderCalibrationSelection(){
        if(calibrationDetail==null)return;JSONObject p=calibrationItem();if(p==null)return;
        String value=p.optString("name")+"  "+calibrationXYZ(p.optJSONArray("xyz"));JSONArray original=p.optJSONArray("original_xyz");
        if(original!=null)value+="\n原始："+calibrationXYZ(original);
        value+="\n"+(calibrationData.optBoolean("dirty")?"有未应用草稿":"与正式路线一致")+" · 修改 "+calibrationData.optInt("modified_points")+" 点 / 插入 "+calibrationData.optInt("control_points")+" 点";
        calibrationDetail.setText(value);
    }
    void calibrationAction(String action,JSONObject extra){
        try{extra.put("route",calibrationRoute).put("revision",calibrationData.getInt("revision")).put("point_id",calibrationPoint);
            sendAction("calibration_"+action,extra,reply->{showCalibration(reply.optJSONObject("calibration"),reply.optString("selected_point",calibrationPoint));message=action.equals("apply")?"校准已应用，进度保留；重新启动 / 恢复导航即可":"校准草稿已保存";Toast.makeText(this,message,Toast.LENGTH_SHORT).show();if(action.equals("apply")||action.equals("discard")){calibrationPreview=false;withMission(m->canvas.invalidate());}},error->{if(calibrationDetail!=null)calibrationDetail.setText(error);Toast.makeText(this,error,Toast.LENGTH_LONG).show();});
        }catch(Exception e){message=e.getMessage();if(calibrationDetail!=null)calibrationDetail.setText(message);}
    }
    void calibrationOffset(String axis,double distance){try{calibrationAction("offset",new JSONObject().put("axis",axis).put("distance",distance));}catch(Exception ignored){}}
    void calibrationCoordinates(){
        JSONObject p=calibrationItem();if(p==null)return;JSONArray xyz=p.optJSONArray("xyz");LinearLayout fields=new LinearLayout(this);fields.setOrientation(1);EditText[] values=new EditText[3];
        for(int i=0;i<3;i++){fields.addView(text(new String[]{"X / m","Y / m","Z / m"}[i],12,WHITE));values[i]=new EditText(this);values[i].setInputType(android.text.InputType.TYPE_CLASS_NUMBER|android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL|android.text.InputType.TYPE_NUMBER_FLAG_SIGNED);values[i].setText(String.format(Locale.ROOT,"%.3f",xyz.optDouble(i)));fields.addView(values[i]);}
        new AlertDialog.Builder(this).setTitle(p.optString("name")+" · 地图坐标").setView(fields).setNegativeButton("取消",null).setPositiveButton("保存草稿",(d,w)->{try{JSONArray point=new JSONArray();for(EditText v:values)point.put(Double.parseDouble(v.getText().toString()));calibrationAction("set",new JSONObject().put("xyz",point));}catch(Exception e){Toast.makeText(this,"请输入有效的 X/Y/Z 坐标",Toast.LENGTH_SHORT).show();}}).show();
    }

    void speedDialog(){
        EditText value=new EditText(this);value.setInputType(android.text.InputType.TYPE_CLASS_NUMBER|android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL);
        JSONObject nav=state.optJSONObject("navigation");value.setText(String.format(Locale.ROOT,"%.2f",nav==null?2.0:nav.optDouble("cruise_mps",2.0)));
        new AlertDialog.Builder(this).setTitle("导航巡航目标 · m/s").setView(value).setNegativeButton("取消",null).setPositiveButton("设置",(d,w)->{
            try{double speed=Double.parseDouble(value.getText().toString());if(!Double.isFinite(speed)||speed<=0)throw new IllegalArgumentException("请输入大于 0 的速度");sendAction("set_speed",new JSONObject().put("cruise_mps",speed));}
            catch(Exception e){message=e.getMessage();}
        }).show();
    }
    void progressDialog(){withMission(m->{try{
        String id=m.getString("route_id");JSONObject route=m.getJSONObject("catalog").getJSONObject(id);JSONArray points=route.getJSONArray("waypoints");
        JSONObject executing=m.getJSONObject("current_route");JSONArray active=executing.getJSONArray("waypoints");
        int target=Math.min(m.optInt("target_index"),active.length()-1);String targetId=active.getJSONObject(target).optString("id");
        int selected=-1;for(int i=0;i<points.length();i++)if(points.getJSONObject(i).optString("id").equals(targetId)){selected=i;break;}
        final int current=selected;String[] labels=new String[points.length()];
        for(int i=0;i<labels.length;i++)labels[i]=(i+1)+" / "+labels.length+" · "+points.getJSONObject(i).optString("name")+(i==current?"  ← 当前目标":"");
        AlertDialog dialog=new AlertDialog.Builder(this).setTitle("设置下一目标 · 之前全部完成").setSingleChoiceItems(labels,current,(d,i)->{
            try{sendAction("set_progress",new JSONObject().put("route",id).put("route_scope","preset").put("target_index",i).put("waypoint_id",points.getJSONObject(i).getString("id")),reply->{
                message="下一目标已设为 "+points.optJSONObject(i).optString("name")+"；之前全部完成，启动后仅执行后续路线";withMission(fresh->canvas.invalidate());
            });d.dismiss();}catch(Exception e){message=e.getMessage();}
        }).setNeutralButton("从头重新开始",(d,w)->{
            try{sendAction("set_progress",new JSONObject().put("route",id).put("route_scope","preset").put("target_index",0).put("waypoint_id",points.getJSONObject(0).getString("id")),reply->{
                message="已清空完成进度，下一目标为 "+points.optJSONObject(0).optString("name")+"；点击启动 / 恢复执行";withMission(fresh->canvas.invalidate());
            });}catch(Exception e){message=e.getMessage();}
        }).setNegativeButton("取消",null).create();dialog.show();if(current>=0)dialog.getListView().setSelection(current);
    }catch(Exception e){message=e.getMessage();}});}
    void navAction(String action,String route){JSONObject extra=new JSONObject();try{if(route!=null)extra.put("route",route);}catch(Exception ignored){}sendAction(action,extra);}
    String actionName(String action){switch(action){case "temporary_append":return "添加临时点";case "temporary_replace":return "替换后续临时点";case "temporary_update":return "修改临时点";case "temporary_remove":return "删除临时点";case "temporary_clear":return "清空临时点";case "pause":return "暂停";case "manual":return "人工控制";case "manual_shadow":return "影子试控";case "override":return "策略选择";case "set_progress":return "设置进度";case "stand":return "起立";case "lie":return "趴下";case "wake":return "唤醒";case "damping":return "阻尼";case "start":return "恢复导航";default:return action;}}
    void sendAction(String action,JSONObject extra){sendAction(action,extra,null);}
    void sendAction(String action,JSONObject extra,Consumer<JSONObject> success){sendAction(action,extra,success,null);}
    void sendAction(String action,JSONObject extra,Consumer<JSONObject> success,Consumer<String> failure){
        if(action.equals("stand")&&manualScreen&&!manualShadow){try{extra.put("manual_after_stand",true).put("policy",manualChoice).put("operator_session",rcSession);}catch(Exception ignored){}}
        if(action.equals("pause")||action.equals("stand")||action.equals("lie")||action.equals("damping")||action.equals("wake"))manualSelection++;
        final String id=UUID.randomUUID().toString();final int selection=manualSelection;
        commands.execute(()->{try{
            if(!visible&&!action.equals("pause"))throw new IOException("App 已离开前台，操作取消");
            JSONObject fresh=rpc("/state",new JSONObject());
            if(!map.getString("asset_id").equals(fresh.optString("asset_id")))throw new IOException("地图版本不匹配");
            extra.put("request_id",id).put("map_id",map.getString("map_id")).put("ticket",fresh.getString("command_ticket")).put("action",action);
            if((action.equals("manual")||action.equals("manual_shadow"))&&selection!=manualSelection)throw new IOException("控制请求已取消");
            JSONObject reply=rpc("/navigation",extra);
            ui.post(()->{JSONObject entry=reply.optJSONObject("entry");message=entry==null?actionName(action)+"已接收；以实时反馈为准":"已规划接入"+(entry.optString("selection").equals("manual_target")?"指定目标 ":"最近前方目标 ")+entry.optString("target_name");if(action.equals("select")||action.equals("set_policy"))withMission(m->canvas.invalidate());if(success!=null)success.accept(reply);});
        }catch(Exception e){ui.post(()->{message=actionName(action)+"："+e.getMessage();if(failure!=null)failure.accept(e.getMessage());});}});
    }
    void emergencyStop(){manualSelection++;new Thread(()->{try{rpc("/stop",new JSONObject());}catch(Exception ignored){}},"operator-stop").start();message="已请求停止并锁定";}
    void withMission(Consumer<JSONObject> callback){io.execute(()->{try{String server=endpoint;JSONObject request=new JSONObject().put("include_catalog",true);JSONObject fresh=rpc("/state",request);if(!server.equals(endpoint))return;if(!map.getString("asset_id").equals(fresh.optString("asset_id")))throw new IOException("地图版本不匹配");JSONObject m=fresh.getJSONObject("mission");cacheRouteDisplay(fresh,server);ui.post(()->{if(!server.equals(endpoint))return;missionData=m;routeDisplaySource="上次同步路线";callback.accept(m);});}catch(Exception e){ui.post(()->message=e.getMessage());}});}
    String routeId(){JSONObject n=state.optJSONObject("navigation");String saved=missionData.optString("route_id","full");return n==null?saved:n.optString("route_id",saved);}
    void policyDialog(){new AlertDialog.Builder(this).setTitle("运控策略").setItems(new String[]{"人工指定（十种策略）","恢复路段预设","编辑分段预设（四种）"},(d,i)->{
        if(i==0)new AlertDialog.Builder(this).setTitle("人工覆盖 · waypoint 进入人工控制").setItems(policyNames,(x,k)->{try{if(k>=6){enterManual(false);chooseManualPolicy(k);}else sendAction("override",new JSONObject().put("policy",policyIds[k]));}catch(Exception ignored){}}).show();
        else if(i==1){try{sendAction("override",new JSONObject().put("policy",JSONObject.NULL));}catch(Exception ignored){}}
        else withMission(this::segmentDialog);
    }).show();}
    void segmentDialog(JSONObject m){try{
        String route=routeId();JSONObject r=m.getJSONObject("catalog").getJSONObject(route);JSONArray edges=r.getJSONArray("edges"),points=r.getJSONArray("waypoints");
        String[] labels=new String[edges.length()];boolean[] chosen=new boolean[labels.length];
        for(int i=0;i<labels.length;i++){String id=edges.getJSONObject(i).optString("policy","basic");String name=policyName(id);
            JSONObject edge=edges.getJSONObject(i);String terrain="";
            for(String key:new String[]{"terrain","anticipated_terrain"}){JSONArray list=edge.optJSONArray(key);if(list!=null)for(int j=0;j<list.length();j++)terrain+=(terrain.length()==0?"\n":"、")+(key.equals("anticipated_terrain")?"提前进入：":"")+list.getJSONObject(j).optString("label");}
            labels[i]=(i+1)+". "+points.getJSONObject(i).optString("name")+" → "+points.getJSONObject(i+1).optString("name")+"  ["+name+"]"+terrain;}
        new AlertDialog.Builder(this).setTitle("选择路段（可多选）").setMultiChoiceItems(labels,chosen,(d,i,on)->chosen[i]=on).setNegativeButton("取消",null).setPositiveButton("选择策略",(d,w)->{
            JSONArray indices=new JSONArray();for(int i=0;i<chosen.length;i++)if(chosen[i])indices.put(i);if(indices.length()==0){message="请至少选择一段";return;}
            new AlertDialog.Builder(this).setTitle("预设官方策略").setItems(Arrays.copyOf(policyNames,navPolicies.length),(x,k)->{try{sendAction("set_policy",new JSONObject().put("route",route).put("segments",indices).put("policy",policyIds[k]));}catch(Exception ignored){}}).show();
        }).show();
    }catch(Exception e){message=e.getMessage();}}
    void replanDialog(){emergencyStop();withMission(m->{try{
        JSONObject r=m.getJSONObject("catalog").getJSONObject(routeId());JSONArray points=r.getJSONArray("waypoints"),edges=r.getJSONArray("edges");
        String[] goals=new String[points.length()];for(int i=0;i<goals.length;i++)goals[i]=i+" · "+points.getJSONObject(i).optString("name");
        new AlertDialog.Builder(this).setTitle("从当前位置重新规划 · 选择目标点").setItems(goals,(d,goal)->{
            String[] labels=new String[edges.length()];boolean[] blocked=new boolean[labels.length];
            for(int i=0;i<labels.length;i++)labels[i]="禁用第 "+(i+1)+" 段";
            new AlertDialog.Builder(this).setTitle("排除阻塞路段（可不选）").setMultiChoiceItems(labels,blocked,(x,i,on)->blocked[i]=on).setNegativeButton("取消",null).setPositiveButton("生成预览",(x,w)->{
                try{JSONArray excluded=new JSONArray();for(int i=0;i<blocked.length;i++)if(blocked[i])excluded.put(i);
                    sendAction("replan",new JSONObject().put("goal",goal).put("blocked_edges",excluded),this::showPreview);
                }catch(Exception e){message=e.getMessage();}
            }).show();
        }).show();
    }catch(Exception e){message=e.getMessage();}});}
    void showPreview(JSONObject reply){try{
        JSONObject p=reply.getJSONObject("preview");missionData.put("preview",p);canvas.invalidate();
        String summary=String.format(Locale.ROOT,"候选路径 %.1f m，%d 段。\n跳过原路线点：%s\n路线提示：%s\n应用后保持暂停，请显式恢复导航。",p.getDouble("length_m"),p.getJSONArray("source_edges").length(),p.getJSONArray("skipped_point_indices"),p.getJSONArray("warnings"));
        new AlertDialog.Builder(this).setTitle("重新规划预览").setMessage(summary).setNegativeButton("放弃",(d,w)->{sendAction("discard_plan",new JSONObject());missionData.remove("preview");canvas.invalidate();}).setPositiveButton("应用路线",(d,w)->{try{sendAction("apply_plan",new JSONObject().put("plan_id",p.getString("id")),r->withMission(m->canvas.invalidate()));}catch(Exception ignored){}}).show();
    }catch(Exception e){message=e.getMessage();}}
    void temporaryMode(int mode){
        temporaryMode=temporaryMode==mode?0:mode;temporaryEditId="";selecting=false;calibrationPreview=false;
        message=temporaryMode==0?"已退出临时路点点选":"点击地图实时指定临时路点";canvas.invalidate();
    }
    void temporaryAction(String operation,JSONObject extra){try{
        extra.put("route",routeId());
        sendAction("temporary_"+operation,extra,reply->{
            JSONObject data=reply.optJSONObject("temporary_waypoints");if(data!=null)temporaryData=data;
            JSONArray points=temporaryData.optJSONArray("pending");message="后续临时点已更新 · "+(points==null?0:points.length())+" 点";
            canvas.invalidate();
        },failure->{message=failure.contains("ALREADY_PASSED_OR_REMOVED")?"该临时点已通过或删除，请选择当前队列中的点":failure.contains("ROUTE_CHANGED")?"当前路线已变化，请重新点选":failure;Toast.makeText(this,message,Toast.LENGTH_SHORT).show();});
    }catch(Exception e){message=e.getMessage();}}
    void temporaryClick(double x,double y){
        if(!connected()){message="未连接 Orin，临时点未发送";return;}
        double z=0,best=Double.POSITIVE_INFINITY;JSONArray pose=state.optJSONArray("pose");
        double currentZ=pose==null?0:pose.optDouble(2);z=currentZ;
        for(double[] a:canvas.anchors)if(canvas.layer(a[2])){
            double distance=Math.hypot(a[0]-x,a[1]-y)+(floorMode==0?.5*Math.abs(a[2]-currentZ):0);
            if(distance<best){best=distance;z=a[2];}
        }
        try{JSONObject extra=new JSONObject().put("xyz",new JSONArray(new double[]{x,y,z}));
            if(temporaryMode==3){extra.put("point_id",temporaryEditId);temporaryAction("update",extra);temporaryMode=0;temporaryEditId="";}
            else temporaryAction(temporaryMode==2?"replace":"append",extra);
        }catch(Exception e){message=e.getMessage();}
    }
    void temporaryDialog(){
        JSONArray points=temporaryData.optJSONArray("pending");if(points==null)points=new JSONArray();final JSONArray items=points;
        String[] labels=new String[items.length()];for(int i=0;i<labels.length;i++){JSONObject point=items.optJSONObject(i);JSONArray xyz=point.optJSONArray("xyz");labels[i]=(i+1)+". "+point.optString("name")+String.format(Locale.ROOT,"  (%.2f, %.2f, %.2f)",xyz.optDouble(0),xyz.optDouble(1),xyz.optDouble(2));}
        AlertDialog.Builder dialog=new AlertDialog.Builder(this).setTitle("未经过的临时点 · 点击修改 / 删除");
        if(labels.length==0)dialog.setMessage("尚无临时点。开启点选追加后，直接点击地图。高度参考附近机身轨迹，可在此修改。");
        else dialog.setItems(labels,(d,index)->temporaryPointDialog(items.optJSONObject(index)));
        dialog.setPositiveButton("点选追加",(d,w)->{temporaryMode=1;temporaryEditId="";selecting=false;calibrationPreview=false;})
            .setNeutralButton("清空临时点",(d,w)->temporaryAction("clear",new JSONObject())).setNegativeButton("关闭",null).show();
    }
    void temporaryPointDialog(JSONObject point){
        new AlertDialog.Builder(this).setTitle(point.optString("name")).setItems(new String[]{"地图改点","编辑坐标 / 高度","删除此点"},(d,which)->{
            if(which==0){temporaryMode=3;temporaryEditId=point.optString("id");selecting=false;calibrationPreview=false;message="点击地图修改 "+point.optString("name");}
            else if(which==2){try{temporaryAction("remove",new JSONObject().put("point_id",point.optString("id")));}catch(Exception ignored){}}
            else{
                LinearLayout box=new LinearLayout(this);box.setOrientation(1);box.setPadding(dp(18),dp(5),dp(18),dp(5));EditText[] values=new EditText[3];JSONArray xyz=point.optJSONArray("xyz");
                for(int i=0;i<3;i++){values[i]=new EditText(this);values[i].setSingleLine(true);values[i].setInputType(12290);values[i].setHint(new String[]{"地图 X (m)","地图 Y (m)","机身地图 Z (m)"}[i]);values[i].setText(String.format(Locale.ROOT,"%.3f",xyz.optDouble(i)));box.addView(values[i]);}
                new AlertDialog.Builder(this).setTitle("编辑临时路点").setView(box).setNegativeButton("取消",null).setPositiveButton("应用",(x,w)->{try{
                    double[] position=new double[3];for(int i=0;i<3;i++){position[i]=Double.parseDouble(values[i].getText().toString());if(!Double.isFinite(position[i]))throw new IllegalArgumentException("坐标须为有效数字");}
                    temporaryAction("update",new JSONObject().put("point_id",point.optString("id")).put("xyz",new JSONArray(position)));
                }catch(Exception e){message=e.getMessage();}}).show();
            }
        }).show();
    }
    void settings(){EditText e=new EditText(this);e.setText(endpoint);e.setSingleLine();new AlertDialog.Builder(this).setTitle("统一导航服务地址").setView(e).setNegativeButton("取消",null).setPositiveButton("保存",(d,w)->{String url=e.getText().toString().trim().replaceAll("/$","");if(!url.startsWith("http://")){message="请输入 http://IP:18894";return;}endpoint=url;getPreferences(0).edit().putString("endpoint",url).apply();received=0;}).show();}
    void coordinateDialog(){LinearLayout box=new LinearLayout(this);box.setPadding(dp(20),dp(8),dp(20),dp(8));box.setOrientation(1);EditText[] fields=new EditText[3];for(int i=0;i<3;i++){fields[i]=new EditText(this);fields[i].setHint(new String[]{"地图 X (m)","地图 Y (m)","机身地图 Z (m)，不是地面高度"}[i]);fields[i].setInputType(12290);if(seed!=null)fields[i].setText(String.format(Locale.ROOT,"%.3f",seed[i]));box.addView(fields[i]);}
        new AlertDialog.Builder(this).setTitle("候选位置 / 高度，不直接设置实际定位").setView(box).setNegativeButton("取消",null).setPositiveButton("设置候选",(d,w)->{try{double[] v=new double[3];for(int i=0;i<3;i++){v[i]=Double.parseDouble(fields[i].getText().toString());if(!Double.isFinite(v[i]))throw new Exception();}seed=v;canvas.invalidate();}catch(Exception e){message="坐标格式不正确";}}).show();}
    void confirmSeed(){if(seed==null){message="请先点选位置";return;}if(!connected()){message="连接有效且地图版本一致后才能提交";return;}
        final double[] xyz=seed.clone();final double r=radius;
        new AlertDialog.Builder(this).setTitle("附近重定位").setMessage(String.format(Locale.ROOT,"以 (%.2f, %.2f, %.2f) 为中心，在 %.0f m 内搜索。\n会暂停导航；更新 Orin 定位，不自动恢复行走。请尽量保持静止数秒。",xyz[0],xyz[1],xyz[2],r)).setNegativeButton("取消",null).setPositiveButton("开始搜索",(d,w)->submit(xyz,r)).show();}
    void submit(double[] xyz,double r){pendingId=UUID.randomUUID().toString();final String id=pendingId;message="正在提交请求 "+id.substring(0,8);io.execute(()->{try{JSONObject fresh=rpc("/state",new JSONObject());if(!map.getString("asset_id").equals(fresh.optString("asset_id")))throw new IOException("地图版本不一致");JSONObject req=new JSONObject().put("map_id",map.getString("map_id")).put("request_id",id).put("ticket",fresh.getString("ticket")).put("xyz",new JSONArray(xyz)).put("radius",r);JSONObject reply=rpc("/relocalize",req);ui.post(()->{message="请求 "+id.substring(0,8)+" 已接收";try{state.put("job",reply.getJSONObject("job"));}catch(Exception ignored){}});}catch(Exception e){ui.post(()->message="请求未确认："+e.getMessage()+"；不会自动重发，请查看状态");}});}
    void cancel(){JSONObject j=state.optJSONObject("job");if(j==null||!(j.optString("state").equals("SEARCHING")||j.optString("state").equals("QUEUED"))){message="没有进行中的搜索";return;}String id=j.optString("request_id");io.execute(()->{try{JSONObject fresh=rpc("/state",new JSONObject());rpc("/cancel",new JSONObject().put("request_id",id).put("ticket",fresh.getString("ticket")));ui.post(()->message="取消请求已接收");}catch(Exception e){ui.post(()->message="取消未确认："+e.getMessage());}});}


    void initRc(){RCSDKManager.INSTANCE.initSDK(getApplicationContext(),new SDKManagerCallBack(){
        public void onRcConnected(){rcConnected=true;rcText="G12 摇杆已连接";}
        public void onRcConnectFail(SkyException e){rcConnected=false;invalidateRc();rcText="摇杆连接失败";}
        public void onRcDisconnect(){rcConnected=false;invalidateRc();rcText="摇杆断开";}
    });RCSDKManager.INSTANCE.setMainThreadCallBack(true);RCSDKManager.INSTANCE.setDebug(false);connectRc();}
    void connectRc(){long now=SystemClock.elapsedRealtime();if(now-lastRcConnect<2000)return;lastRcConnect=now;RCSDKManager.INSTANCE.connectToRC();}
    void invalidateRc(){synchronized(rcLock){rcAt=0;rcEpoch++;rcTicket++;rcReading=false;}}
    static double norm(int v){double d=(v-1500)/450.;return Math.abs(d)<.08?0:Math.copySign(Math.min(1,(Math.abs(d)-.08)/.92),d);}
    final Runnable rcPoll=new Runnable(){public void run(){
        if(visible&&!rcReading){
            // SDK connectToRC() returns silently when its shared product is
            // already connected. Read channels without waiting for a new callback.
            if(!rcConnected)connectRc();
            rcReading=true;rcRequest=SystemClock.elapsedRealtime();final long epoch=rcEpoch,start=rcRequest,requestId=++rcTicket;
            KeyManager.INSTANCE.get(RemoteControllerKey.INSTANCE.getKeyChannels(),new CompletionCallbackWith<int[]>(){
                boolean completed=false;
                public void onSuccess(int[] v){if(completed||epoch!=rcEpoch||requestId!=rcTicket)return;completed=true;rcReading=false;long now=SystemClock.elapsedRealtime();
                    if(!visible||epoch!=rcEpoch||now-start>250||v==null||v.length<16){invalidateRc();return;}
                    for(int i:new int[]{0,1,2,3,6,7,8,9})if(v[i]<900||v[i]>2100){invalidateRc();return;}
                    synchronized(rcLock){
                        rcConnected=true;
                        if(v[9]>=1900||(rcAt!=0&&v[7]>=1900&&channels[7]<1900))emergencyStop();
                        else if(rcAt!=0&&connected()){
                            if(v[6]>=1900&&channels[6]<1900)enterManual(true);
                            if(v[8]>=1900&&channels[8]<1900)sendAction("stand",new JSONObject());
                        }
                        channels=v.clone();rcAt=now;rcSeq++;rcText=String.format(Locale.ROOT,"G12 前后 %.2f 横移 %.2f 转向 %.2f",norm(v[2]),-norm(v[3]),-norm(v[0]));
                    }
                }
                public void onFailure(SkyException e){if(completed||epoch!=rcEpoch||requestId!=rcTicket)return;completed=true;rcConnected=false;invalidateRc();rcText="摇杆读取失败";android.util.Log.w("GOAINavRC",String.valueOf(e));}
            });
        }
        if(rcReading&&SystemClock.elapsedRealtime()-rcRequest>300)invalidateRc();ui.postDelayed(this,100);
    }};

    final class MapView extends View {
        Paint paint=new Paint(Paint.ANTI_ALIAS_FLAG);float[] points;double[][] anchors;JSONArray route,indoor;float cx=0,cy=0,scale=5;boolean fitted=false,pinched=false;float lx,ly,downX,downY;long downTime;
        ScaleGestureDetector zoom;
        Bitmap cloudLayer;float cachedCx=Float.NaN,cachedCy,cachedScale;int cachedFloor=-1;
        void drawCloud(Canvas target){
            if(cloudLayer==null||cloudLayer.getWidth()!=getWidth()||cloudLayer.getHeight()!=getHeight()){
                if(cloudLayer!=null)cloudLayer.recycle();cloudLayer=Bitmap.createBitmap(getWidth(),getHeight(),Bitmap.Config.ARGB_8888);cachedCx=Float.NaN;
            }
            if(cachedCx!=cx||cachedCy!=cy||cachedScale!=scale||cachedFloor!=floorMode){
                cloudLayer.eraseColor(Color.TRANSPARENT);Canvas cloud=new Canvas(cloudLayer);
                float[] drawn=new float[points.length/3*2];int n=0;
                for(int i=0;i<points.length;i+=3){if(!layer(points[i+2]))continue;float x=sx(points[i]),y=sy(points[i+1]);if(x>=0&&x<=getWidth()&&y>=0&&y<=getHeight()){drawn[n++]=x;drawn[n++]=y;}}
                paint.setStyle(Paint.Style.FILL);paint.setColor(Color.rgb(75,100,115));paint.setStrokeWidth(dp(1));cloud.drawPoints(drawn,0,n,paint);
                cachedCx=cx;cachedCy=cy;cachedScale=scale;cachedFloor=floorMode;
            }
            target.drawBitmap(cloudLayer,0,0,null);
        }
        MapView(){super(MainActivity.this);setBackgroundColor(Color.rgb(13,27,39));
            JSONArray a=map.optJSONArray("points");points=new float[a.length()*3];for(int i=0;i<a.length();i++){JSONArray p=a.optJSONArray(i);for(int k=0;k<3;k++)points[i*3+k]=(float)p.optDouble(k);}
            a=map.optJSONArray("anchors");anchors=new double[a.length()][3];for(int i=0;i<a.length();i++)for(int k=0;k<3;k++)anchors[i][k]=a.optJSONArray(i).optDouble(k);
            route=map.optJSONArray("waypoints");indoor=map.optJSONArray("indoor");
            zoom=new ScaleGestureDetector(MainActivity.this,new ScaleGestureDetector.SimpleOnScaleGestureListener(){public boolean onScale(ScaleGestureDetector d){float wx=worldX(d.getFocusX()),wy=worldY(d.getFocusY());scale=Math.max(.4f,Math.min(dp(140),scale*d.getScaleFactor()));cx=wx-(d.getFocusX()-getWidth()/2f)/scale;cy=wy+(d.getFocusY()-getHeight()/2f)/scale;follow=false;pinched=true;invalidate();return true;}});
        }
        float sx(double x){return (float)((x-cx)*scale+getWidth()/2f);}float sy(double y){return (float)(getHeight()/2f-(y-cy)*scale);}
        float worldX(float x){return cx+(x-getWidth()/2f)/scale;}float worldY(float y){return cy-(y-getHeight()/2f)/scale;}
        boolean layer(double z){return floorMode==0||(floorMode==1?z<2:z>=2);}
        void fit(){if(getWidth()==0)return;JSONArray b=map.optJSONArray("bounds");double x0=Math.min(0,b.optJSONArray(0).optDouble(0)),y0=Math.min(0,b.optJSONArray(0).optDouble(1)),x1=Math.max(0,b.optJSONArray(1).optDouble(0)),y1=Math.max(0,b.optJSONArray(1).optDouble(1));cx=(float)((x0+x1)/2);cy=(float)((y0+y1)/2);scale=(float)Math.min((getWidth()-dp(50))/(x1-x0+8),(getHeight()-dp(40))/(y1-y0+8));fitted=true;invalidate();}
        @Override protected void onSizeChanged(int w,int h,int ow,int oh){if(!fitted)fit();}
        void line(Canvas c,double x0,double y0,double x1,double y1,int color,float width){paint.setColor(color);paint.setStrokeWidth(width);c.drawLine(sx(x0),sy(y0),sx(x1),sy(y1),paint);}
        @Override protected void onDraw(Canvas c){super.onDraw(c);JSONArray pose=state.optJSONArray("pose");if(follow&&pose!=null&&connected()){cx=(float)pose.optDouble(0);cy=(float)pose.optDouble(1);}
            drawCloud(c);paint.setStyle(Paint.Style.FILL);
            JSONObject catalog=missionData.optJSONObject("catalog");
            JSONObject selectedRoute=catalog==null?null:catalog.optJSONObject(routeId());
            JSONObject preview=missionData.optJSONObject("preview");JSONObject previewRoute=preview==null?null:preview.optJSONObject("route");
            boolean draft=calibrationPreview&&calibrationRoute.equals(routeId());if(temporaryMode!=0)previewRoute=null;
            routeViewButton.setText(draft?"返回正式路线":"正式路线");
            if(draft){
                JSONArray items=calibrationData.optJSONArray("items");drawRoute(c,items,Color.rgb(229,143,242));
            }else if(previewRoute!=null){drawProgressRoute(c,previewRoute);}
            else if(selectedRoute!=null){drawProgressRoute(c,selectedRoute);}
            else if(missionData.optString("route_id").equals(routeId())){
                JSONObject current=missionData.optJSONObject("current_route");if(current!=null)drawProgressRoute(c,current);
            }
            // Recorded traces belong to the explicit recording view/session.
            JSONArray recordedTrace=(recordingWindow!=null&&recordingWindow.isShowing())?recordingData.optJSONArray("trace"):null;
            if(recordedTrace!=null)for(int i=1;i<recordedTrace.length();i++){JSONArray a=recordedTrace.optJSONObject(i-1).optJSONArray("xyz"),b=recordedTrace.optJSONObject(i).optJSONArray("xyz");if(layer(a.optDouble(2))||layer(b.optDouble(2)))line(c,a.optDouble(0),a.optDouble(1),b.optDouble(0),b.optDouble(1),Color.rgb(240,125,220),dp(3));}
            JSONObject nav=state.optJSONObject("navigation");if(nav!=null&&!draft&&previewRoute==null){
                JSONObject entry=nav.optJSONObject("entry");JSONArray path=entry==null?null:entry.optJSONArray("path");
                if(path!=null)for(int i=Math.max(1,entry.optInt("leg",1));i<path.length();i++){JSONArray a=path.optJSONArray(i-1),b=path.optJSONArray(i);if(layer(a.optDouble(2))||layer(b.optDouble(2)))line(c,a.optDouble(0),a.optDouble(1),b.optDouble(0),b.optDouble(1),Color.rgb(230,145,255),dp(3));}
                JSONObject returning=nav.optJSONObject("temporary_return");JSONArray target=returning==null?nav.optJSONArray("target_xyz"):returning.optJSONArray("xyz");if(target!=null&&layer(target.optDouble(2))){paint.setColor(Color.rgb(255,120,108));paint.setStyle(Paint.Style.STROKE);paint.setStrokeWidth(dp(2));c.drawCircle(sx(target.optDouble(0)),sy(target.optDouble(1)),dp(9),paint);paint.setStyle(Paint.Style.FILL);}}
            if(!draft&&previewRoute==null){
                JSONArray pending=temporaryData.optJSONArray("pending"),previous=pose;paint.setTextSize(dp(12));
                if(pending!=null)for(int i=0;i<pending.length();i++){
                    JSONObject point=pending.optJSONObject(i);JSONArray xyz=point.optJSONArray("xyz");if(xyz==null)continue;
                    if(previous!=null&&(layer(previous.optDouble(2))||layer(xyz.optDouble(2)))){paint.setPathEffect(new DashPathEffect(new float[]{dp(7),dp(5)},0));line(c,previous.optDouble(0),previous.optDouble(1),xyz.optDouble(0),xyz.optDouble(1),GOLD,dp(3));paint.setPathEffect(null);}
                    if(layer(xyz.optDouble(2))){paint.setColor(GOLD);c.drawCircle(sx(xyz.optDouble(0)),sy(xyz.optDouble(1)),dp(6),paint);c.drawText(point.optString("name"),sx(xyz.optDouble(0))+dp(8),sy(xyz.optDouble(1))-dp(8),paint);}previous=xyz;
                }
            }
            JSONArray trace=state.optJSONArray("track");if(trace!=null){JSONObject prev=null;for(int i=0;i<trace.length();i++){JSONObject t=trace.optJSONObject(i);JSONArray xyz=t.optJSONArray("xyz");if(prev!=null&&t.optString("epoch").equals(prev.optString("epoch"))&&t.optDouble("t")-prev.optDouble("t")<2&&layer(xyz.optDouble(2))){JSONArray p=prev.optJSONArray("xyz");line(c,p.optDouble(0),p.optDouble(1),xyz.optDouble(0),xyz.optDouble(1),Color.rgb(137,224,144),dp(2));}prev=t;}}
            paint.setColor(TEAL);c.drawCircle(sx(0),sy(0),dp(4),paint);paint.setTextSize(dp(10));c.drawText("录制起点",sx(0)+dp(5),sy(0)-dp(5),paint);
            if(seed!=null){paint.setColor(Color.rgb(204,145,255));paint.setStyle(Paint.Style.STROKE);paint.setStrokeWidth(dp(2));c.drawCircle(sx(seed[0]),sy(seed[1]),(float)(radius*scale),paint);c.drawLine(sx(seed[0])-dp(8),sy(seed[1]),sx(seed[0])+dp(8),sy(seed[1]),paint);c.drawLine(sx(seed[0]),sy(seed[1])-dp(8),sx(seed[0]),sy(seed[1])+dp(8),paint);paint.setStyle(Paint.Style.FILL);}
            if(pose!=null){float x=sx(pose.optDouble(0)),y=sy(pose.optDouble(1));boolean valid=connected()&&state.optBoolean("valid");paint.setColor(valid?Color.rgb(93,245,164):Color.GRAY);c.drawCircle(x,y,dp(6),paint);double yaw=pose.optDouble(3);paint.setStrokeWidth(dp(3));c.drawLine(x,y,x+(float)Math.cos(yaw)*dp(21),y-(float)Math.sin(yaw)*dp(21),paint);c.drawText(valid?"机器人":"最后位置（未确认）",x+dp(10),y+dp(14),paint);}
            double meters=scale>dp(20)?1:scale>dp(5)?5:10;paint.setColor(WHITE);paint.setStrokeWidth(dp(2));c.drawLine(dp(14),getHeight()-dp(15),dp(14)+(float)(meters*scale),getHeight()-dp(15),paint);paint.setTextSize(dp(11));c.drawText((int)meters+" m",dp(14),getHeight()-dp(22),paint);
        }
        void drawProgressRoute(Canvas c,JSONObject routeData){
            JSONArray points=routeData.optJSONArray("waypoints"),edges=routeData.optJSONArray("edges");
            if(points==null)return;JSONObject nav=state.optJSONObject("navigation");JSONArray reached=nav==null?null:nav.optJSONArray("reached_indices");
            Set<String> done=new HashSet<>();JSONObject execution=missionData.optJSONObject("current_route");
            JSONArray executed=execution==null?null:execution.optJSONArray("waypoints");
            if(reached!=null&&executed!=null)for(int i=0;i<reached.length();i++){JSONObject w=executed.optJSONObject(reached.optInt(i));if(w!=null)done.add(w.optString("id",w.optString("name")));}
            for(int i=0;i<points.length();i++){JSONObject w=points.optJSONObject(i);JSONArray p=w.optJSONArray("xyz");if(p==null)continue;
                boolean completed=done.contains(w.optString("id",w.optString("name")));int color=completed?TEAL:Color.rgb(120,180,255);
                if(i>0){JSONArray a=points.optJSONObject(i-1).optJSONArray("xyz");JSONObject edge=edges==null?null:edges.optJSONObject(i-1);JSONArray controls=edge==null?null:edge.optJSONArray("control_points");
                    if(controls!=null)for(int j=0;j<controls.length();j++){JSONArray q=controls.optJSONObject(j).optJSONArray("xyz");if(a!=null&&q!=null){line(c,a.optDouble(0),a.optDouble(1),q.optDouble(0),q.optDouble(1),color,dp(2));paint.setColor(TEAL);c.drawCircle(sx(q.optDouble(0)),sy(q.optDouble(1)),dp(4),paint);a=q;}}
                    if(a!=null&&(layer(a.optDouble(2))||layer(p.optDouble(2))))line(c,a.optDouble(0),a.optDouble(1),p.optDouble(0),p.optDouble(1),color,dp(2));}
                if(layer(p.optDouble(2))){paint.setColor(color);c.drawCircle(sx(p.optDouble(0)),sy(p.optDouble(1)),dp(3),paint);if(scale>dp(7)){paint.setTextSize(dp(10));c.drawText((completed?"✓ ":"")+w.optString("name"),sx(p.optDouble(0))+dp(4),sy(p.optDouble(1))-dp(4),paint);}}
            }
        }
        void drawRoute(Canvas c,JSONArray route,int color){if(route==null)return;for(int i=0;i<route.length();i++){JSONObject w=route.optJSONObject(i);JSONArray p=w.optJSONArray("xyz");if(p==null)continue;if(i>0){JSONArray prev=route.optJSONObject(i-1).optJSONArray("xyz");if(prev!=null&&(layer(p.optDouble(2))||layer(prev.optDouble(2))))line(c,prev.optDouble(0),prev.optDouble(1),p.optDouble(0),p.optDouble(1),color,dp(2));}if(layer(p.optDouble(2))){paint.setColor(color);c.drawCircle(sx(p.optDouble(0)),sy(p.optDouble(1)),dp(3),paint);if(scale>dp(7)){paint.setTextSize(dp(10));String name=w.optString("name",w.optString("id"));if(name.length()>10)name=name.substring(0,10);c.drawText(name,sx(p.optDouble(0))+dp(4),sy(p.optDouble(1))-dp(4),paint);}}}}
        @Override public boolean onTouchEvent(MotionEvent e){zoom.onTouchEvent(e);float x=e.getX(),y=e.getY();switch(e.getActionMasked()){
            case MotionEvent.ACTION_DOWN:lx=downX=x;ly=downY=y;downTime=SystemClock.elapsedRealtime();pinched=false;return true;
            case MotionEvent.ACTION_POINTER_DOWN:pinched=true;return true;
            case MotionEvent.ACTION_MOVE:if(e.getPointerCount()==1&&!zoom.isInProgress()&&!pinched){cx-=(x-lx)/scale;cy+=(y-ly)/scale;follow=false;invalidate();}lx=x;ly=y;return true;
            case MotionEvent.ACTION_UP:if(temporaryMode!=0&&!pinched&&Math.hypot(x-downX,y-downY)<dp(9)&&SystemClock.elapsedRealtime()-downTime<600){temporaryClick(worldX(x),worldY(y));performClick();invalidate();return true;}if(selecting&&!pinched&&Math.hypot(x-downX,y-downY)<dp(9)&&SystemClock.elapsedRealtime()-downTime<600){double wx=worldX(x),wy=worldY(y),best=Double.POSITIVE_INFINITY,z=0;for(double[] a:anchors)if(layer(a[2])){double d=Math.hypot(a[0]-wx,a[1]-wy);if(d<best){best=d;z=a[2];}}seed=new double[]{wx,wy,z};message=best>5?"此处离参考采集轨迹较远，建议换点":"候选点已选择；高度取最近的参考机身轨迹，可调整";performClick();invalidate();}return true;
            default:return true;}}
        @Override public boolean performClick(){super.performClick();return true;}
    }
}

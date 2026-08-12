package com.alwaysbea.esc;

import android.Manifest;
import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.net.ConnectivityManager;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.net.NetworkRequest;
import android.net.Uri;
import android.net.wifi.WifiConfiguration;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.provider.Settings;
import android.text.InputType;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ListView;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.List;

/**
 * 开始界面：输入/选择 WiFi 热点 -> 连接 -> 进入 WebView 控制页。
 * 连接策略：
 *  - Android 10+ (API>=29)：WifiNetworkSpecifier（系统弹窗确认，无需定位权限）
 *  - Android 7~9 (API 24-28)：WifiManager.addNetwork + enableNetwork
 */
public class MainActivity extends Activity {

    static final String DEFAULT_URL = "http://192.168.4.1";
    private static final int REQ_LOC = 100;
    private static final int REQ_WRITE_SETTINGS = 101;

    private EditText etSsid, etPwd;
    private TextView tvStatus;
    private ListView list;
    private ArrayAdapter<String> adapter;
    private final List<String> ssids = new ArrayList<>();

    private WifiManager wifi;
    private ConnectivityManager cm;
    private Handler h = new Handler();

    private ConnectivityManager.NetworkCallback netCb;
    private NetworkRequest netReq;
    private String pendingSsid, pendingPwd;
    /* 仅 Android 7~9 legacy 连接流程中，等待 supplicant COMPLETED 后自动进入控制页；
     * 防止系统/其他 WiFi 的网络变化把用户无端拉进控制页 */
    private boolean legacyConnecting = false;

    private final BroadcastReceiver scanReceiver = new BroadcastReceiver() {
        @Override public void onReceive(Context c, Intent i) {
            ssids.clear();
            try {
                List<android.net.wifi.ScanResult> r = wifi.getScanResults();
                java.util.Collections.sort(r, (a, b) ->
                        Integer.compare(b.level, a.level));
                for (android.net.wifi.ScanResult s : r) {
                    if (s.SSID != null && !s.SSID.isEmpty() && !ssids.contains(s.SSID)) {
                        ssids.add(s.SSID);
                    }
                }
            } catch (Exception ignored) {
                /* 定位权限未授予时系统会拒绝 getScanResults，忽略即可 */
            }
            adapter.notifyDataSetChanged();
            setStatus("扫描到 " + ssids.size() + " 个网络");
        }
    };

    private final BroadcastReceiver stateReceiver = new BroadcastReceiver() {
        @Override public void onReceive(Context c, Intent i) {
            if (WifiManager.SUPPLICANT_STATE_CHANGED_ACTION.equals(i.getAction())) {
                android.net.wifi.SupplicantState st;
                try {
                    if (Build.VERSION.SDK_INT >= 33) {
                        st = i.getParcelableExtra(WifiManager.EXTRA_NEW_STATE,
                                android.net.wifi.SupplicantState.class);
                    } else {
                        st = i.getParcelableExtra(WifiManager.EXTRA_NEW_STATE);
                    }
                } catch (Exception e) {
                    st = null;
                }
                if (st == null) return;
                setStatus("WiFi 状态: " + st.toString());
                if (st == android.net.wifi.SupplicantState.COMPLETED && legacyConnecting) {
                    legacyConnecting = false;
                    h.postDelayed(() -> openControl(), 800);
                }
            }
        }
    };

    @Override
    protected void onCreate(Bundle b) {
        super.onCreate(b);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        wifi = (WifiManager) getApplicationContext().getSystemService(WIFI_SERVICE);
        cm = (ConnectivityManager) getApplicationContext().getSystemService(CONNECTIVITY_SERVICE);

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        int pad = dp(20);
        root.setPadding(pad, pad, pad, pad);

        TextView title = new TextView(this);
        title.setText("⚓ AlwaysbeA 控制");
        title.setTextSize(22);
        root.addView(title);

        TextView sub = new TextView(this);
        sub.setText("连接设备热点后自动进入控制界面");
        sub.setTextSize(13);
        root.addView(sub);

        root.addView(label("WiFi 名称（设备热点）"));
        etSsid = new EditText(this);
        etSsid.setText("AlwaysbeA");
        root.addView(etSsid);

        root.addView(label("WiFi 密码"));
        etPwd = new EditText(this);
        etPwd.setText("12345678");
        etPwd.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_PASSWORD);
        root.addView(etPwd);

        Button btnConnect = new Button(this);
        btnConnect.setText("连接并进入控制");
        root.addView(btnConnect);

        Button btnScan = new Button(this);
        btnScan.setText("扫描附近 WiFi 选择设备");
        root.addView(btnScan);

        tvStatus = new TextView(this);
        tvStatus.setText("未连接");
        tvStatus.setTextSize(14);
        root.addView(tvStatus);

        list = new ListView(this);
        adapter = new ArrayAdapter<>(this, android.R.layout.simple_list_item_1, ssids);
        list.setAdapter(adapter);
        root.addView(list, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1f));

        setContentView(root);

        btnConnect.setOnClickListener(v ->
                connect(etSsid.getText().toString().trim(), etPwd.getText().toString()));
        btnScan.setOnClickListener(v -> scan());
        list.setOnItemClickListener((p, view, pos, id) -> {
            String ssid = ssids.get(pos);
            etSsid.setText(ssid);
            connect(ssid, etPwd.getText().toString());
        });

        registerReceiver(scanReceiver,
                new IntentFilter(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION));
        registerReceiver(stateReceiver,
                new IntentFilter(WifiManager.SUPPLICANT_STATE_CHANGED_ACTION));

        if (Build.VERSION.SDK_INT < 29) {
            wifi.setWifiEnabled(true);
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        unregisterReceiver(scanReceiver);
        unregisterReceiver(stateReceiver);
        if (netCb != null && netReq != null && Build.VERSION.SDK_INT >= 29) {
            try { cm.unregisterNetworkCallback(netCb); } catch (Exception ignored) {}
        }
    }

    private void scan() {
        if (Build.VERSION.SDK_INT >= 23 && checkSelfPermission(
                Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{
                    Manifest.permission.ACCESS_FINE_LOCATION,
                    Manifest.permission.ACCESS_COARSE_LOCATION}, REQ_LOC);
            return;
        }
        setStatus("扫描中…");
        if (Build.VERSION.SDK_INT >= 23 && !wifi.isWifiEnabled()) {
            wifi.setWifiEnabled(true);
        }
        wifi.startScan();
    }

    @Override
    public void onRequestPermissionsResult(int code, String[] perm, int[] gr) {
        super.onRequestPermissionsResult(code, perm, gr);
        if (code == REQ_LOC) scan();
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == REQ_WRITE_SETTINGS) {
            if (Settings.System.canWrite(this)) {
                setStatus("已授予，重试连接…");
                if (pendingSsid != null) connect(pendingSsid, pendingPwd);
            } else {
                setStatus("未授予「修改系统设置」，无法连接（可到系统设置中开启后重试）");
            }
        }
    }

    private void connect(String ssid, String pwd) {
        if (ssid.isEmpty()) { setStatus("请输入 WiFi 名称"); return; }
        /* 自检：CHANGE_NETWORK_STATE 是普通权限，应随安装自动授予。
         * 若未授予，说明当前安装的还是旧版 APK（清单里没有该权限），提示卸载重装。 */
        if (Build.VERSION.SDK_INT >= 23 &&
                checkSelfPermission(Manifest.permission.CHANGE_NETWORK_STATE)
                        != PackageManager.PERMISSION_GRANTED) {
            setStatus("系统权限缺失：请先卸载本应用，再安装最新版 APK");
            return;
        }
        pendingSsid = ssid;
        pendingPwd = pwd;
        try {
            if (Build.VERSION.SDK_INT >= 29) {
                connectSpecifier(ssid, pwd);
            } else {
                connectLegacy(ssid, pwd);
            }
        } catch (SecurityException e) {
            /* Android 14 要求 CHANGE_NETWORK_STATE 或 WRITE_SETTINGS 二选一。
             * 若系统仍拒绝（部分厂商 ROM 不认可普通权限），引导授予 WRITE_SETTINGS 后重试。 */
            if (Build.VERSION.SDK_INT >= 23 && !Settings.System.canWrite(this)) {
                setStatus("系统要求授予「修改系统设置」权限…");
                Intent it = new Intent(Settings.ACTION_MANAGE_WRITE_SETTINGS,
                        Uri.parse("package:" + getPackageName()));
                if (it.resolveActivity(getPackageManager()) != null) {
                    startActivityForResult(it, REQ_WRITE_SETTINGS);
                } else {
                    setStatus("连接出错: " + e.getClass().getSimpleName() + ": " + e.getMessage());
                }
            } else {
                setStatus("连接出错: " + e.getClass().getSimpleName() + ": " + e.getMessage());
            }
        } catch (Exception e) {
            setStatus("连接出错: " + e.getClass().getSimpleName() + ": " + e.getMessage());
        }
    }

    /** Android 10+：WifiNetworkSpecifier，系统弹窗确认，连接后绑定进程并打开网页 */
    private void connectSpecifier(String ssid, String pwd) {
        setStatus("请求连接 " + ssid + " …（请留意系统弹窗）");
        legacyConnecting = false;
        if (netCb != null) {
            try { cm.unregisterNetworkCallback(netCb); } catch (Exception ignored) {}
        }
        android.net.wifi.WifiNetworkSpecifier.Builder b =
                new android.net.wifi.WifiNetworkSpecifier.Builder()
                        .setSsid(ssid);
        if (pwd != null && !pwd.isEmpty()) {
            if (pwd.length() < 8) {
                setStatus("密码至少 8 位（WPA2）");
                return;
            }
            b.setWpa2Passphrase(pwd);
        }
        netReq = new NetworkRequest.Builder()
                .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
                .setNetworkSpecifier(b.build())
                .build();
        netCb = new ConnectivityManager.NetworkCallback() {
            @Override public void onAvailable(Network network) {
                runOnUiThread(() -> {
                    setStatus("已连接 " + ssid + "，进入控制界面");
                    if (Build.VERSION.SDK_INT >= 23) {
                        cm.bindProcessToNetwork(network);
                    }
                    openControl();
                });
            }
            @Override public void onUnavailable() {
                runOnUiThread(() -> setStatus("连接失败/被拒绝，请重试"));
            }
        };
        cm.requestNetwork(netReq, netCb);
    }

    /** Android 7~9：addNetwork + enableNetwork，等待 supplicant 完成 */
    private void connectLegacy(String ssid, String pwd) {
        setStatus("连接 " + ssid + " …");
        legacyConnecting = true;
        WifiConfiguration cfg = new WifiConfiguration();
        cfg.SSID = "\"" + ssid + "\"";
        cfg.status = WifiConfiguration.Status.ENABLED;
        if (pwd == null || pwd.isEmpty()) {
            cfg.allowedKeyManagement.set(WifiConfiguration.KeyMgmt.NONE);
        } else {
            cfg.allowedKeyManagement.set(WifiConfiguration.KeyMgmt.WPA_PSK);
            cfg.allowedProtocols.set(WifiConfiguration.Protocol.WPA);
            cfg.allowedProtocols.set(WifiConfiguration.Protocol.RSN);
            cfg.allowedPairwiseCiphers.set(WifiConfiguration.PairwiseCipher.TKIP);
            cfg.allowedPairwiseCiphers.set(WifiConfiguration.PairwiseCipher.CCMP);
            cfg.allowedGroupCiphers.set(WifiConfiguration.GroupCipher.TKIP);
            cfg.allowedGroupCiphers.set(WifiConfiguration.GroupCipher.CCMP);
            cfg.preSharedKey = "\"" + pwd + "\"";
        }
        int id = wifi.addNetwork(cfg);
        if (id < 0) { setStatus("添加网络失败"); return; }
        wifi.disconnect();
        wifi.enableNetwork(id, true);
        wifi.reconnect();
        h.postDelayed(() -> {
            WifiInfo info = wifi.getConnectionInfo();
            if (info != null && info.getSSID() != null &&
                    info.getSSID().contains(ssid)) {
                legacyConnecting = false;
                openControl();
            } else {
                legacyConnecting = false;
                setStatus("尚未连上 " + ssid + "，正在等待…");
            }
        }, 4000);
    }

    private void openControl() {
        Intent it = new Intent(this, ControlActivity.class);
        it.putExtra("url", DEFAULT_URL);
        startActivity(it);
    }

    private void setStatus(String s) {
        tvStatus.setText(s);
    }

    private TextView label(String s) {
        TextView t = new TextView(this);
        t.setText(s);
        t.setTextSize(13);
        return t;
    }

    private int dp(int v) {
        return (int) (v * getResources().getDisplayMetrics().density + 0.5f);
    }

    private String stateName(int state) {
        switch (state) {
            case 0: return "扫描中";
            case 1: return "连接中";
            case 2: return "认证中";
            case 3: return "已关联";
            case 4: return "已完成";
            default: return "未知(" + state + ")";
        }
    }
}

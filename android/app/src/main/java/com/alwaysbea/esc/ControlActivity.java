package com.alwaysbea.esc;

import android.app.Activity;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.view.View;
import android.webkit.JavascriptInterface;
import android.webkit.WebResourceRequest;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import java.io.OutputStream;
import java.nio.charset.StandardCharsets;

/**
 * 控制界面：WebView 加载 ESP32 上的控制网页（192.168.4.1）。
 * 顶部提供「返回设备选择」与「刷新」按钮。
 */
public class ControlActivity extends Activity {

    private static final int REQUEST_EXPORT = 1001;
    private static final int MAX_EXPORT_CHARS = 2 * 1024 * 1024;
    private WebView web;
    private String pendingExportName;
    private String pendingExportMime;
    private String pendingExportData;

    private final class ExportBridge {
        @JavascriptInterface
        public void saveText(String fileName, String mimeType, String data) {
            runOnUiThread(() -> {
                if (!isTrustedDevicePage()) return;
                openExportPicker(fileName, mimeType, data);
            });
        }
    }

    @Override
    protected void onCreate(Bundle b) {
        super.onCreate(b);

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);

        LinearLayout bar = new LinearLayout(this);
        bar.setOrientation(LinearLayout.HORIZONTAL);
        Button btnBack = new Button(this);
        btnBack.setText("← 返回");
        btnBack.setOnClickListener(v -> finish());
        Button btnReload = new Button(this);
        btnReload.setText("刷新");
        btnReload.setOnClickListener(v -> web.reload());
        TextView tv = new TextView(this);
        tv.setText("AlwaysbeA 控制");
        tv.setPadding(dp(12), 0, dp(12), 0);
        tv.setTextSize(18);
        bar.addView(btnBack);
        bar.addView(tv, new LinearLayout.LayoutParams(0, -2, 1f));
        bar.addView(btnReload);
        root.addView(bar);

        web = new WebView(this);
        WebSettings s = web.getSettings();
        s.setJavaScriptEnabled(true);
        s.setDomStorageEnabled(true);
        s.setCacheMode(WebSettings.LOAD_DEFAULT);
        /* 用短 UA 减小请求头体积：ESP32 httpd 默认只收 512B 请求头，
         * 系统 WebView 的完整 UA 很容易超限导致 "Header fields are too long" */
        s.setUserAgentString("AlwaysbeA/1.0");
        /* 页面运行在设备本地热点；导出桥只打开系统另存为窗口，不上传数据。 */
        web.addJavascriptInterface(new ExportBridge(), "AlwaysbeABridge");
        web.setWebViewClient(new WebViewClient() {
            @Override
            public boolean shouldOverrideUrlLoading(WebView view, WebResourceRequest request) {
                return !isTrustedDeviceUri(request.getUrl());
            }

            @SuppressWarnings("deprecation")
            @Override
            public boolean shouldOverrideUrlLoading(WebView view, String targetUrl) {
                return !isTrustedDeviceUri(Uri.parse(targetUrl));
            }

            @Override
            public void onReceivedError(WebView view, int code, String desc, String url) {
                // 显示重试界面
                view.loadDataWithBaseURL(null,
                        "<html><body style='text-align:center;padding-top:60px;color:#888;'>"
                        + "无法连接设备 (" + code + ")<br>请确认已连接设备热点后点刷新"
                        + "</body></html>", "text/html", "utf-8", null);
            }
        });
        root.addView(web, new LinearLayout.LayoutParams(-1, -1, 1f));
        setContentView(root);

        String url = getIntent().getStringExtra("url");
        if (url == null) url = MainActivity.DEFAULT_URL;
        web.loadUrl(url);
    }

    private void openExportPicker(String fileName, String mimeType, String data) {
        if (data == null) return;
        if (pendingExportData != null) {
            Toast.makeText(this, "已有导出任务正在等待保存", Toast.LENGTH_SHORT).show();
            return;
        }
        if (data.length() > MAX_EXPORT_CHARS) {
            Toast.makeText(this, "导出数据过大，请减少历史记录后重试", Toast.LENGTH_LONG).show();
            return;
        }
        pendingExportName = (fileName == null || fileName.trim().isEmpty())
                ? "alwaysbea-training-history.json" : fileName;
        pendingExportName = pendingExportName.replaceAll("[^a-zA-Z0-9._-]", "_");
        pendingExportMime = mimeType != null && mimeType.toLowerCase().startsWith("text/csv")
                ? "text/csv" : "application/json";
        pendingExportData = data;

        Intent intent = new Intent(Intent.ACTION_CREATE_DOCUMENT);
        intent.addCategory(Intent.CATEGORY_OPENABLE);
        intent.setType(pendingExportMime);
        intent.putExtra(Intent.EXTRA_TITLE, pendingExportName);
        try {
            startActivityForResult(intent, REQUEST_EXPORT);
        } catch (Exception e) {
            clearPendingExport();
            Toast.makeText(this, "当前系统无法打开文件保存窗口", Toast.LENGTH_LONG).show();
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode != REQUEST_EXPORT) return;
        if (resultCode != RESULT_OK || data == null || data.getData() == null ||
                pendingExportData == null) {
            clearPendingExport();
            return;
        }

        Uri uri = data.getData();
        try (OutputStream out = getContentResolver().openOutputStream(uri, "w")) {
            if (out == null) throw new IllegalStateException("openOutputStream returned null");
            out.write(pendingExportData.getBytes(StandardCharsets.UTF_8));
            out.flush();
            Toast.makeText(this, "训练历史已保存", Toast.LENGTH_SHORT).show();
        } catch (Exception e) {
            Toast.makeText(this, "保存失败：" + e.getMessage(), Toast.LENGTH_LONG).show();
        } finally {
            clearPendingExport();
        }
    }

    private void clearPendingExport() {
        pendingExportName = null;
        pendingExportMime = null;
        pendingExportData = null;
    }

    private boolean isTrustedDevicePage() {
        if (web == null || web.getUrl() == null) return false;
        try {
            return isTrustedDeviceUri(Uri.parse(web.getUrl()));
        } catch (Exception ignored) {
            return false;
        }
    }

    private boolean isTrustedDeviceUri(Uri uri) {
        return uri != null && "http".equalsIgnoreCase(uri.getScheme()) &&
                "192.168.4.1".equals(uri.getHost());
    }

    @Override
    public void onBackPressed() {
        if (web != null && web.canGoBack()) {
            web.goBack();
        } else {
            super.onBackPressed();
        }
    }

    private int dp(int v) {
        return (int) (v * getResources().getDisplayMetrics().density + 0.5f);
    }
}

package com.alwaysbea.esc;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

/**
 * 控制界面：WebView 加载 ESP32 上的控制网页（192.168.4.1）。
 * 顶部提供「返回设备选择」与「刷新」按钮。
 */
public class ControlActivity extends Activity {

    private WebView web;

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
        web.setWebViewClient(new WebViewClient() {
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

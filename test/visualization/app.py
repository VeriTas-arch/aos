from flask import Flask, render_template, jsonify
import subprocess

app = Flask(__name__)

# 设置静态文件夹，用来提供HTML文件
app.static_folder = "static"


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/open-cmd", methods=["POST"])
def open_cmd():
    subprocess.Popen(["cmd", "/K", "echo hello"])
    subprocess.Popen(
        ["powershell", "-NoExit", "-Command", "Write-Host 'hello'"],
        creationflags=subprocess.CREATE_NEW_CONSOLE,
    )
    # subprocess.Popen(
    #     [
    #         "powershell",
    #         "-NoExit",
    #         "-Command",
    #         "start 'foxglove://open?ds=rosbridge-websocket&ds.url=ws://localhost:9090&openIn=desktop'",
    #     ],
    #     creationflags=subprocess.CREATE_NEW_CONSOLE,
    # )
    return jsonify(message="CMD window opened with 'hello'")


if __name__ == "__main__":
    app.run(debug=True)

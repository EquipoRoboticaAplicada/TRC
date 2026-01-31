from flask import Flask, Response, request, jsonify
from video_stream import gen_frames
from gpiozero import DigitalOutputDevice
import time

# ----- GPIO -----

IN1 = DigitalOutputDevice(23)

def signal_on():
    IN1.on()
    

def signal_off():
    IN1.off()
    
# ----- Estado de Visión -----

vision_state= {"colores":[],"centroides":[],"areas":[],"time": 0.0}



# ----- Flask -----
app = Flask(__name__)


@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/command", methods=["POST"])
def command():
    data = request.json
    cmd = data.get("cmd")

    if cmd == "signal_on":
        signal_on()
    elif cmd == "signal_off":
        signal_off()

    return jsonify({"status": "ok", "cmd": cmd})

@app.route("/vision_data",methods=["POST"])

def vision_data():
    global vision_state
    
    data=request.json
    vision_state=["colores"]=data.get("colores",[])
    vision_state=["centroides"]=data.get("centroides",[])
    vision_state=["areas"]=data.get("areas",[])
    vision_state=["time"]=data.get("time",time.time())
    
    return jsonify({"status":"ok"})
    

if __name__ == "__main__":
    signal_off()
    app.run(host="0.0.0.0", port=5000)
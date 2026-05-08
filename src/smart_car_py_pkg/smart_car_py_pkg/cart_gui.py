import socket
import io
import qrcode
import time
from flask import Flask, render_template_string, send_file, jsonify, request


# =========================================================
# 1. 데이터 관리 클래스 (Cart Manager)
# =========================================================
class CartManager:
    """YOLO 인식 데이터를 직접 받아 장바구니 상태를 관리합니다."""

    def __init__(self):
        self.cart_items = []
        self.payment_completed = False
        self.last_state = {"length": 0, "qty_sum": 0}

    def add_item(self, class_name):
        """YOLO에서 전달받은 클래스명으로 상품을 추가합니다. (중복 인식 시 자동 증가 방지)"""
        item_id = class_name.strip().lower()

        # 1. YOLO 인식 클래스명(영문/한글)을 고정된 한글 상품명으로 매핑
        name_map = {
            "suncream": "선크림",
            "선크림": "선크림",
            "tape": "테이프",
            "테이프": "테이프",
            "scissor": "가위",
            "scissors": "가위",
            "가위": "가위",
            "tissue": "물티슈",
            "물티슈": "물티슈",
            "wipes": "물티슈",
            "wet wipes": "물티슈",
        }
        display_name = name_map.get(item_id, item_id.upper())

        # 이미 장바구니에 있는 품목이면 수량을 올리지 않고 무시함 (수동 조절만 허용)
        for item in self.cart_items:
            if item["id"] == item_id:
                print(f"[장바구니 유지] {display_name} 이미 존재함 (자동 수량 증가 차단)")
                return True

        # 2. 물품별 개별 가격 설정
        price_map = {
            "선크림": 12000,
            "테이프": 1000,
            "가위": 1000,
            "물티슈": 2000,
        }
        item_price = price_map.get(display_name, 2000)

        # 3. 물품별 고정 상품 이미지 링크
        img_map = {
            "선크림": "https://thumbnail.coupangcdn.com/thumbnails/remote/657x657q90trim/image/retail/images/711495719789444-63608ebf-36e9-49df-ba97-eba04797f78b.jpg",
            "테이프": "https://thumbnail.coupangcdn.com/thumbnails/remote/657x657q90trim/image/retail/images/930519213985665-402cf826-9ad2-4213-a040-f1151bf3f30f.jpg",
            "가위": "https://thumbnail.coupangcdn.com/thumbnails/remote/230x230ex/image/vendor_inventory/302f/cfbc5ddcb3fb7aa9ee76a6b09ed2f1803c89d01d49ad6e533e45f839de63.jpg",
            "물티슈": "https://thumbnail.coupangcdn.com/thumbnails/remote/657x657q90trim/image/retail/images/2025/05/15/11/4/d65a6c9d-dc29-41c7-9c64-df83841ee3cd.jpg",
        }
        img_url = img_map.get(display_name, f"https://placehold.co/150x150/EEEEEE/333333?text={display_name}")

        self.cart_items.append({
            "id": item_id,
            "name": display_name,
            "price": item_price,
            "qty": 1,
            "img_url": img_url,
        })
        print(f"[장바구니 추가] {display_name} 최초 1개 담김 (가격: {item_price}원)")
        return True

    def update_qty(self, class_name, delta):
        """UI에서 수동으로 상품의 수량을 증감(또는 삭제)합니다."""
        item_id = class_name.strip().lower()

        for item in self.cart_items:
            if item["id"] == item_id:
                item["qty"] += delta
                if item["qty"] <= 0:
                    self.cart_items.remove(item)
                    print(f"[장바구니 삭제] {item['name']} 항목 완전 제거됨")
                else:
                    print(f"[장바구니 수동 업데이트] {item['name']} 수량 변경 (현재: {item['qty']}개)")
                return True
        return False

    def reset(self):
        self.cart_items = []
        self.payment_completed = False

    def check_changed(self):
        curr_len = len(self.cart_items)
        curr_qty = sum(item["qty"] for item in self.cart_items)
        changed = (curr_len != self.last_state["length"] or curr_qty != self.last_state["qty_sum"])
        self.last_state = {"length": curr_len, "qty_sum": curr_qty}
        return changed


# =========================================================
# 2. 시스템 유틸리티 클래스
# =========================================================
class SystemUtils:
    """네트워크 IP 및 QR 코드 생성을 담당합니다."""

    @staticmethod
    def get_local_ip():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("10.255.255.255", 1))
            ip = s.getsockname()[0]
        except Exception:
            ip = "127.0.0.1"
        finally:
            s.close()
        return ip

    @staticmethod
    def create_qr_buffer(url):
        qr = qrcode.QRCode(version=1, box_size=10, border=4)
        qr.add_data(url)
        qr.make(fit=True)
        img = qr.make_image(fill_color="black", back_color="white")
        buf = io.BytesIO()
        img.save(buf, format="PNG")
        buf.seek(0)
        return buf


# =========================================================
# 3. 템플릿 엔진 클래스 (UI/UX 담당)
# =========================================================
class TemplateManager:
    HEAD_COMMON = """
    <meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/css/bootstrap.min.css" rel="stylesheet">
    <style>
        body { background-color: #eaeded; font-family: -apple-system, sans-serif; padding-bottom: 120px; }
        .navbar-custom { background-color: #232f3e; color: white; padding: 15px; text-align: center; font-weight: 700; font-size: 1.4rem; }
        .promo-banner { background-color: #e3f2fd; color: #0d47a1; padding: 12px; text-align: center; font-weight: bold; font-size: 0.95rem; }
        .cart-container { background-color: white; margin-top: 15px; padding: 20px; border-radius: 8px; box-shadow: 0 1px 3px rgba(0,0,0,0.1); }
        .product-card { display: flex; border-bottom: 1px solid #ddd; padding-bottom: 20px; margin-bottom: 20px; }
        .product-img { width: 90px; height: 90px; object-fit: contain; border-radius: 8px; margin-right: 15px; background-color: #f8f9fa; padding: 8px; }
        .fixed-bottom-bar { position: fixed; bottom: 0; left: 0; width: 100%; background-color: white; padding: 15px 25px; box-shadow: 0 -4px 20px rgba(0,0,0,0.1); z-index: 1030; display: flex; justify-content: space-between; align-items: center; border-radius: 20px 20px 0 0; }
        .total-price { font-size: 1.8rem; font-weight: 900; color: #B12704; }
        .btn-pay { font-size: 1.2rem; padding: 12px 25px; border-radius: 50px; background-color: #ffd814; border: 1px solid #fcd200; font-weight: bold; }
        .qty-input { width: 50px; text-align: center; font-weight: bold; border-left: none; border-right: none; background-color: white !important; }
    </style>
    """

    MAIN = """
    <!DOCTYPE html><html><head>""" + HEAD_COMMON + """<title>Smart Cart [VERIFIED_CODE]</title></head>
    <body>
        <nav class="navbar-custom">Frictionless Store</nav>
        <div class="promo-banner"> [오픈 이벤트] 전 품목 20% 할인 진행 중!</div>
        <div class="container-fluid">
            <div class="cart-container">
                <h5 class="fw-bold border-bottom pb-2 mb-3">장바구니 <small class="text-muted fw-normal">자동 인식 대기중 (카메라 켜짐)</small></h5>
                {% if items %}
                    {% for item in items %}
                    <div class="product-card">
                        <img src="{{ item.img_url }}" class="product-img" referrerpolicy="no-referrer" onerror="this.src='https://placehold.co/90x90/EEEEEE/333333?text={{ item.name }}'">
                        <div class="flex-grow-1">
                            <div class="fw-bold fs-5">{{ item.name }}</div>
                            <div class="text-success small mb-2">YOLO 인식 완료</div>
                            <div class="d-flex justify-content-between align-items-center">
                                <span class="fs-5 fw-bold">{{ "{:,}".format(item.price * item.qty) }}원</span>
                                <div class="input-group input-group-sm" style="width: 120px;">
                                    <button class="btn btn-outline-secondary px-3" onclick="updateQty('{{ item.id }}', -1)">-</button>
                                    <input type="text" class="form-control qty-input" value="{{ item.qty }}" readonly>
                                    <button class="btn btn-outline-secondary px-3" onclick="updateQty('{{ item.id }}', 1)">+</button>
                                </div>
                            </div>
                        </div>
                    </div>
                    {% endfor %}
                {% else %}
                    <div class="text-center py-5 text-muted">장바구니가 비어 있습니다.<br><br>카메라에 물건을 보여주세요.</div>
                {% endif %}
            </div>
        </div>

        {% set total_qty = items|sum(attribute='qty') %}
        {% set total_price = namespace(value=0) %}
        {% for item in items %}{% set total_price.value = total_price.value + (item.price * item.qty) %}{% endfor %}

        <div class="fixed-bottom-bar">
            <div><div class="small text-muted">총 수량: {{ total_qty }}개</div><div class="total-price">{{ "{:,}".format(total_price.value) }}원</div></div>
            <button class="btn btn-pay" onclick="new bootstrap.Modal(document.getElementById('qrModal')).show(); document.getElementById('qrImg').src='/qrcode?t='+Date.now()"> 결제하기</button>
        </div>

        <div class="modal fade" id="qrModal" tabindex="-1" aria-hidden="true">
            <div class="modal-dialog modal-dialog-centered"><div class="modal-content border-0 rounded-4 text-center shadow-lg"><div class="modal-body p-5">
                <h4 class="fw-bold mb-3">결제 QR 코드</h4><p class="text-muted">스마트폰으로 스캔하여 결제하세요.</p>
                <div class="p-3 bg-light rounded-4 d-inline-block border"><img id="qrImg" src="" style="width: 200px;"></div>
                <div class="mt-4"><button class="btn btn-outline-dark rounded-pill px-5" data-bs-dismiss="modal">취소</button></div>
            </div></div></div>
        </div>

        <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/js/bootstrap.bundle.min.js"></script>
        <script>
            function updateQty(className, delta) {
                fetch('/api/update_qty', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ class_name: className, delta: delta })
                }).then(response => {
                    if (response.ok) {
                        window.location.reload();
                    }
                });
            }

            setInterval(() => {
                fetch('/api/status').then(r => r.json()).then(data => {
                    if(data.cart_changed) window.location.reload();
                    if(data.paid) window.location.href='/show_success';
                });
            }, 1000);
        </script>
    </body></html>
    """

    RECEIPT = """
    <!DOCTYPE html><html><head>""" + HEAD_COMMON + """<title>Receipt</title>
    <style>.receipt-card { max-width: 400px; margin: 30px auto; border-radius: 12px; border: 1px solid #ddd; background: white; }</style></head>
    <body class="p-3">
        <div class="receipt-card p-4 shadow-sm text-center">
            <div class="display-4 text-success mb-2">OK</div><h3 class="fw-bold">주문 완료</h3>
            <div class="border-top border-bottom py-3 my-3 text-start">
                {% for item in items %}<div class="d-flex justify-content-between"><span>{{item.name}} ({{item.qty}})</span><b>{{ "{:,}".format(item.price*item.qty) }}원</b></div>{% endfor %}
            </div>
            <div class="d-flex justify-content-between fs-4 fw-bold"><span>총액</span><span class="text-danger">{{ "{:,}".format(total) }}원</span></div>
            <button class="btn btn-dark w-100 mt-4 py-3 fw-bold" onclick="closeApp()">확인</button>
        </div>
        <script>
            function closeApp() {
                const ua = navigator.userAgent.toLowerCase();
                if(ua.includes('kakaotalk')) location.href='kakaotalk://inappbrowser/close';
                else if(ua.includes('naver')) location.href='naversearchapp://inappbrowser/close';
                else { window.open('','_self').close(); setTimeout(()=>{ location.href='/thank_you' },500); }
            }
        </script>
    </body></html>
    """

    SUCCESS_POPUP = """
    <!DOCTYPE html><html><head>""" + HEAD_COMMON + """<title>Success</title></head>
    <body>
        <div class="modal fade show" style="display:block; background:rgba(0,0,0,0.5);" tabindex="-1">
            <div class="modal-dialog modal-dialog-centered"><div class="modal-content border-0 rounded-4 text-center shadow-lg"><div class="modal-body p-5">
                <div style="font-size: 50px; color: #007600; margin-bottom: 15px;">OK</div>
                <h4 class="fw-bold mb-3">결제가 완료되었습니다</h4>
                <p class="text-muted mb-4">이용해 주셔서 감사합니다.<br>안전하게 상품을 담아가세요.</p>
                <button class="btn btn-dark rounded-pill px-5 py-2 fw-bold" onclick="window.location.href='/reset_cart'">확인</button>
            </div></div></div>
        </div>
    </body></html>
    """


# =========================================================
# 4. 서버 메인 엔진 클래스
# =========================================================
class SmartCartServer:
    """Flask 서버 구동 및 API 라우팅을 총괄합니다."""

    def __init__(self):
        self.app = Flask(__name__)
        self.cart = CartManager()
        self.utils = SystemUtils()
        self.ui = TemplateManager()
        self.host_ip = self.utils.get_local_ip()

        self._setup_routes()

    def _setup_routes(self):
        @self.app.after_request
        def add_header(r):
            r.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
            return r

        @self.app.route("/")
        def index():
            self.cart.check_changed()
            return render_template_string(self.ui.MAIN, items=self.cart.cart_items)

        @self.app.route("/qrcode")
        def qrcode_gen():
            url = f"http://{self.host_ip}:5000/process_payment?t={int(time.time())}"
            return send_file(self.utils.create_qr_buffer(url), mimetype="image/png")

        @self.app.route("/process_payment")
        def process():
            total = sum(i["price"] * i["qty"] for i in self.cart.cart_items)
            res = render_template_string(self.ui.RECEIPT, items=self.cart.cart_items, total=total)
            self.cart.payment_completed = True
            return res

        @self.app.route("/api/status")
        def status():
            return jsonify({"paid": self.cart.payment_completed, "cart_changed": self.cart.check_changed()})

        @self.app.route("/api/add_item", methods=["POST"])
        def api_add():
            data = request.json
            if not data:
                return jsonify({"status": "fail"}), 400
            c_name = data.get("class_name")
            if c_name and self.cart.add_item(c_name):
                return jsonify({"status": "ok"})
            return jsonify({"status": "fail"}), 400

        @self.app.route("/api/update_qty", methods=["POST"])
        def api_update_qty():
            data = request.json
            if not data:
                return jsonify({"status": "fail"}), 400
            c_name = data.get("class_name")
            delta = data.get("delta", 0)
            if c_name and self.cart.update_qty(c_name, delta):
                return jsonify({"status": "ok"})
            return jsonify({"status": "fail"}), 400

        @self.app.route("/show_success")
        def show_success():
            return render_template_string(self.ui.SUCCESS_POPUP)

        @self.app.route("/reset_cart")
        def reset():
            self.cart.reset()
            return """<script>window.location.href='/';</script>"""

        @self.app.route("/thank_you")
        def thanks():
            return "<body style='text-align:center;padding-top:100px;'><h3>결제가 완료되었습니다.</h3><p>화면을 닫아주세요.</p></body>"

    def run(self):
        print(f"\n{'=' * 45}\n Smart Cart [VERIFIED_CODE] Server Online\n API: http://{self.host_ip}:5000/api/add_item\n GUI: http://{self.host_ip}:5000\n Recognition: ROS2 bridge\n{'=' * 45}")
        self.app.run(host="0.0.0.0", port=5000, debug=False)


def main():
    server = SmartCartServer()
    server.run()


if __name__ == "__main__":
    main()

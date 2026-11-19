"""
teleop/utils/ssl_certs.py
Automatic self-signed SSL certificate generation with SAN for HTTPS / WSS WebXR server.
Includes Subject Alternative Names (SAN) for localhost, 127.0.0.1, ::1, and LAN IP.
"""

import os
import socket
import subprocess
import tempfile
from pathlib import Path


def get_local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def _cert_covers_ip(cert_path: Path, ip: str) -> bool:
    """True if the cert's SAN lists `ip` (or the IP cannot be determined, so we never regenerate needlessly)."""
    if ip in ("127.0.0.1", ""):
        return True
    try:
        out = subprocess.run(["openssl", "x509", "-in", str(cert_path), "-noout", "-ext", "subjectAltName"],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return True
    return f"IP Address:{ip}" in out if out else True


def ensure_ssl_certs(certs_dir: str = None) -> tuple[str, str]:
    """
    Ensures that cert.pem and key.pem exist in the given directory with proper SAN.
    Keeps the same cert across runs so Quest Browser only needs to accept once (regenerated only when the LAN IP changes).
    Returns (cert_path, key_path).
    """
    if certs_dir is None:
        project_root = Path(__file__).resolve().parent.parent.parent
        certs_dir = project_root / "certs"
    else:
        certs_dir = Path(certs_dir)

    certs_dir.mkdir(parents=True, exist_ok=True)
    cert_path = certs_dir / "cert.pem"
    key_path = certs_dir / "key.pem"

    if cert_path.exists() and key_path.exists() and not _cert_covers_ip(cert_path, get_local_ip()):
        print(f"[INFO] LAN IP changed ({get_local_ip()}) and is missing from the certificate SAN: regenerating.")
        cert_path.unlink(); key_path.unlink()

    if not cert_path.exists() or not key_path.exists():
        print(f"[INFO] Generating self-signed SSL certificates with SAN in {certs_dir}...")
        lan_ip = get_local_ip()

        # Build OpenSSL configuration with SAN
        san_config = f"""
[req]
distinguished_name = req_distinguished_name
x509_extensions = v3_req
prompt = no

[req_distinguished_name]
CN = localhost

[v3_req]
keyUsage = critical, digitalSignature, keyEncipherment
extendedKeyUsage = serverAuth
subjectAltName = @alt_names

[alt_names]
DNS.1 = localhost
IP.1 = 127.0.0.1
IP.2 = ::1
IP.3 = {lan_ip}
"""
        with tempfile.NamedTemporaryFile("w", delete=False) as f:
            f.write(san_config)
            conf_file = f.name

        try:
            cmd = [
                "openssl", "req", "-x509", "-newkey", "rsa:2048",
                "-keyout", str(key_path),
                "-out", str(cert_path),
                "-days", "3650", "-nodes",
                "-config", conf_file
            ]
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("[INFO] Self-signed SSL certificate with SAN generated successfully.")
            print("[NOTE] Quest Browser requires accepting certificate once (Advanced -> Proceed to localhost).")
        except Exception as e:
            print(f"[WARN] Failed to generate SSL certificate via openssl: {e}")
        finally:
            if os.path.exists(conf_file):
                os.remove(conf_file)

    return str(cert_path), str(key_path)

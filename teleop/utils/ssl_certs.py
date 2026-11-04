"""
teleop/utils/ssl_certs.py
Automatic self-signed SSL certificate generation for HTTPS / WSS WebXR server.
"""

import os
import subprocess
from pathlib import Path

def ensure_ssl_certs(certs_dir: str = None) -> tuple[str, str]:
    """
    Ensures that cert.pem and key.pem exist in the given directory.
    If missing, automatically generates a self-signed certificate using openssl.
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

    if not cert_path.exists() or not key_path.exists():
        print(f"[INFO] Generating self-signed SSL certificates in {certs_dir}...")
        cmd = [
            "openssl", "req", "-x509", "-newkey", "rsa:2048",
            "-keyout", str(key_path),
            "-out", str(cert_path),
            "-days", "365", "-nodes",
            "-subj", "/CN=localhost"
        ]
        try:
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("[INFO] Self-signed SSL certificate generated successfully.")
        except Exception as e:
            print(f"[WARN] Failed to generate SSL certificate via openssl: {e}")

    return str(cert_path), str(key_path)

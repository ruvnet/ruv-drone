use ruv_navigation_lab::{evaluate, simulate, Scenario};
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::time::{Duration, Instant};

const ADDR: &str = "127.0.0.1:8777";
const HTML: &str = include_str!("../web/index.html");
const JS: &str = include_str!("../web/app.js");
const CSS: &str = include_str!("../web/style.css");

fn response(stream: &mut TcpStream, status: &str, kind: &str, body: &str) -> std::io::Result<()> {
    write!(stream,"HTTP/1.1 {status}\r\nContent-Type: {kind}\r\nContent-Length: {}\r\nConnection: close\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\nX-Frame-Options: DENY\r\nReferrer-Policy: no-referrer\r\nContent-Security-Policy: default-src 'none'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'; base-uri 'none'; form-action 'none'\r\n\r\n{body}",body.len())
}

fn authorized_head(head: &str) -> bool {
    let mut hosts = 0;
    for line in head.lines().skip(1) {
        if line.is_empty() {
            break;
        }
        let Some((key, value)) = line.split_once(':') else {
            return false;
        };
        let value = value.trim();
        if key.eq_ignore_ascii_case("host") {
            hosts += 1;
            if !["127.0.0.1:8777", "localhost:8777"].contains(&value) {
                return false;
            }
        }
        if key.eq_ignore_ascii_case("origin")
            && !["http://127.0.0.1:8777", "http://localhost:8777"].contains(&value)
        {
            return false;
        }
        if key.eq_ignore_ascii_case("content-length") && value != "0" {
            return false;
        }
        if key.eq_ignore_ascii_case("transfer-encoding") {
            return false;
        }
    }
    hosts == 1
}
fn request_run(path: &str) -> Option<(Scenario, u32)> {
    let query = path.strip_prefix("/api/run?")?;
    let mut scenario = None;
    let mut seed = None;
    for field in query.split('&') {
        let (k, v) = field.split_once('=')?;
        match k {
            "scenario" if scenario.is_none() => scenario = Some(Scenario::parse(v)?),
            "seed" if seed.is_none() => seed = Some(v.parse::<u32>().ok()?),
            _ => return None,
        }
    }
    Some((scenario?, seed?))
}
fn serve(mut stream: TcpStream, report: &str) -> std::io::Result<()> {
    stream.set_read_timeout(Some(Duration::from_millis(500)))?;
    stream.set_write_timeout(Some(Duration::from_secs(2)))?;
    let start = Instant::now();
    let mut bytes = Vec::with_capacity(4096);
    while !bytes.ends_with(b"\r\n\r\n") {
        if bytes.len() >= 4096 || start.elapsed() > Duration::from_secs(1) {
            return response(
                &mut stream,
                "413 Payload Too Large",
                "text/plain",
                "Request limit",
            );
        }
        let mut b = [0u8; 1];
        if stream.read(&mut b)? == 0 {
            return Ok(());
        }
        bytes.push(b[0]);
    }
    let Ok(head) = std::str::from_utf8(&bytes) else {
        return response(
            &mut stream,
            "400 Bad Request",
            "text/plain",
            "Invalid request",
        );
    };
    if !authorized_head(head) {
        return response(
            &mut stream,
            "403 Forbidden",
            "text/plain",
            "Local origin required",
        );
    }
    let parts: Vec<_> = head
        .lines()
        .next()
        .unwrap_or("")
        .split_whitespace()
        .collect();
    if parts.len() != 3 || parts[0] != "GET" || parts[2] != "HTTP/1.1" {
        return response(
            &mut stream,
            "405 Method Not Allowed",
            "text/plain",
            "GET only",
        );
    }
    match parts[1] {
        "/" => response(&mut stream, "200 OK", "text/html; charset=utf-8", HTML),
        "/app.js" => response(&mut stream, "200 OK", "text/javascript; charset=utf-8", JS),
        "/style.css" => response(&mut stream, "200 OK", "text/css; charset=utf-8", CSS),
        "/api/evaluate" => response(&mut stream, "200 OK", "application/json", report),
        "/health" => response(
            &mut stream,
            "200 OK",
            "application/json",
            "{\"ok\":true,\"simulation_only\":true}",
        ),
        path if path.starts_with("/api/run?") => match request_run(path) {
            Some((scenario, seed)) => response(
                &mut stream,
                "200 OK",
                "application/json",
                &simulate(seed, scenario).json(),
            ),
            None => response(
                &mut stream,
                "400 Bad Request",
                "text/plain",
                "Invalid scenario or seed",
            ),
        },
        _ => response(&mut stream, "404 Not Found", "text/plain", "Not found"),
    }
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("evaluate")=>{let(ok,report)=evaluate();println!("{report}");if !ok{std::process::exit(1);}},
        Some("run")=>{
            let scenario=args.get(1).and_then(|s|Scenario::parse(s)).ok_or("run requires scenario and optional u32 seed")?;
            let seed=args.get(2).map(|s|s.parse::<u32>()).transpose()?.unwrap_or(42);
            println!("{}",simulate(seed,scenario).json());
        },
        Some("serve")|None=>{
            let listener=TcpListener::bind(ADDR)?;
            let(ok,report)=evaluate();if !ok{return Err("Startup validation failed".into());}
            eprintln!("ruv-drone Navigation Lab: http://{ADDR} | 100/100 scenarios passed | simulation only");
            for stream in listener.incoming() {match stream {Ok(s)=>{if let Err(e)=serve(s,&report){eprintln!("Request ended: {e}");}},Err(e)=>eprintln!("Accept failed: {e}")}}
        },
        _=>return Err("Usage: ruv-navigation-lab [serve | evaluate | run <inspection|blocked|stale|link|unknown> [seed]]".into()),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rejects_rebinding_and_cross_origin() {
        assert!(authorized_head(
            "GET / HTTP/1.1\r\nHost: 127.0.0.1:8777\r\n\r\n"
        ));
        assert!(!authorized_head(
            "GET / HTTP/1.1\r\nHost: attacker.example\r\n\r\n"
        ));
        assert!(!authorized_head(
            "GET / HTTP/1.1\r\nHost: localhost:8777\r\nOrigin: https://attacker.example\r\n\r\n"
        ));
        assert!(!authorized_head(
            "GET / HTTP/1.1\r\nHost: localhost:8777\r\nHost: localhost:8777\r\n\r\n"
        ));
        assert!(!authorized_head(
            "GET / HTTP/1.1\r\nHost: localhost:8777\r\nContent-Length: 10\r\n\r\n"
        ));
    }
    #[test]
    fn bounded_typed_query() {
        assert_eq!(
            request_run("/api/run?scenario=inspection&seed=42"),
            Some((Scenario::Inspection, 42))
        );
        for bad in [
            "/api/run?scenario=bad&seed=42",
            "/api/run?scenario=inspection&seed=-1",
            "/api/run?scenario=inspection&seed=4294967296",
            "/api/run?scenario=inspection&seed=1&seed=2",
            "/api/run?scenario=inspection&seed=1&file=/etc/passwd",
        ] {
            assert!(request_run(bad).is_none());
        }
    }
}

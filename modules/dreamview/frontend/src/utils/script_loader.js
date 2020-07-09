export default function loadScriptAsync({ url, onLoad, onError }) {
  const script = document.createElement('script');
  script.src = url;
  script.type = 'text/javascript';
  script.async = true;
  script.onload = onLoad;
  script.onerror = onError;
  document.body.appendChild(script);
}

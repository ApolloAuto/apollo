class ImagePreview {
  static init() {
    let moved = false;

    // create image preview modal
    let modal = document.createElement('div');
    modal.className = 'image-preview-modal';
    modal.style.display = 'none';
    modal.style.position = 'fixed';
    modal.style.width = '100%';
    modal.style.height = '100%';
    modal.style.backgroundColor = 'rgba(0, 0, 0, 0.5)';
    let modalContainer = document.createElement('div');
    modalContainer.className = 'image-preview-modal-container';
    // modal.appendChild(modalContainer);
    let modalImage = document.createElement('img');
    modalImage.className = 'image-preview-modal-image';
    modalImage.draggable = false;
    modal.appendChild(modalImage);

    modal.addEventListener('click', function () {
      if (!moved) {
        modal.style.display = 'none';
      }
    });
    modalContainer.addEventListener('wheel', function (e) {
      if (e.deltaY > 0) {
        //
      } else if (e.deltaY < 0) {
        //
      }
    });

    modal.addEventListener('wheel', function (e) {
      let modalHeight = modal.clientHeight;
      let modalWidth = modal.clientWidth;
      let realHeight = modalImage.naturalHeight;
      let realWidth = modalImage.naturalWidth;

      let ratio = modalImage.width / modalImage.naturalWidth;

      if (e.deltaY > 0) {
        ratio += 0.1;
        if (ratio > 1) {
          ratio = 1;
        }
      } else if (e.deltaY < 0) {
        ratio -= 0.1;
        if (ratio < 0.2) {
          ratio = 0.2;
        }
      }

      modalImage.style.maxHeight =
        ((realHeight * ratio) / modalHeight) * 100 + '%';
      modalImage.style.maxWidth =
        ((realWidth * ratio) / modalWidth) * 100 + '%';

      // reset to center
      modalImage.style.top = (modalHeight - realHeight * ratio) / 2 + 'px';
      modalImage.style.left = (modalWidth - realWidth * ratio) / 2 + 'px';
    });

    modalImage.addEventListener('mousedown', function (evt) {
      evt.preventDefault();
      let x = evt.clientX;
      let y = evt.clientY;
      let top = modalImage.offsetTop;
      let left = modalImage.offsetLeft;
      moved = false;

      // TODO: add mousemove event listener
      document.onmousemove = function (e) {
        moved = true;
        let dx = e.clientX - x;
        let dy = e.clientY - y;

        modalImage.style.top = top + dy + 'px';
        modalImage.style.left = left + dx + 'px';

        e.target.style.cursor = 'move';
      };

      // TODO: add mouseup event listener
      document.onmouseup = function (e) {
        document.onmousemove = null;
        document.onmouseup = null;
        e.target.style.cursor = 'default';
      };
    });

    // add image click event listener
    window.addEventListener('load', function () {
      document.querySelectorAll('img').forEach(function (img) {
        img.addEventListener('click', function () {
          let realHeight = img.naturalHeight;
          let realWidth = img.naturalWidth;

          let modalHeight = document.body.clientHeight;
          let modalWidth = document.body.clientWidth;

          let ratioHeight = realHeight / modalHeight;
          let ratioWidth = realWidth / modalWidth;

          if (ratioHeight > ratioWidth) {
            let top = modalHeight * 0.2;
            let ratio = (modalHeight * 0.6) / realHeight;

            // modalImage.style.top = top + 'px';

            if (ratio < 1) {
              modalImage.style.maxHeight = '60%';
              modalImage.style.maxWidth =
                ((realWidth * ratio) / modalWidth) * 100 + '%';
              modalImage.style.top =
                (modalHeight - realHeight * ratio) / 2 + 'px';
              modalImage.style.left =
                (modalWidth - realWidth * ratio) / 2 + 'px';
            } else {
              modalImage.style.maxHeight =
                (realHeight / modalHeight) * 100 + '%';
              modalImage.style.maxWidth = (realWidth / modalWidth) * 100 + '%';
              modalImage.style.top = (modalHeight - realHeight) / 2 + 'px';
              modalImage.style.left = (modalWidth - realWidth) / 2 + 'px';
            }
          } else {
            let left = modalWidth * 0.2;
            let ratio = (modalWidth * 0.6) / realWidth;

            // modalImage.style.left = left + 'px';

            if (ratio < 1) {
              modalImage.style.maxWidth = '60%';
              modalImage.style.maxHeight =
                ((realHeight * ratio) / modalHeight) * 100 + '%';
              modalImage.style.top =
                (modalHeight - realHeight * ratio) / 2 + 'px';
              modalImage.style.left =
                (modalWidth - realWidth * ratio) / 2 + 'px';
            } else {
              modalImage.style.maxWidth = (realWidth / modalWidth) * 100 + '%';
              modalImage.style.maxHeight =
                (realHeight / modalHeight) * 100 + '%';
              modalImage.style.top = (modalHeight - realHeight) / 2 + 'px';
              modalImage.style.left = (modalWidth - realWidth) / 2 + 'px';
            }
          }

          modalImage.src = img.src;
          modal.style.display = 'block';
        });
      });
      document.body.insertBefore(modal, document.body.firstChild);
    });
  }
}

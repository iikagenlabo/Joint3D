//	キー入力.
var KeyInput = function() {
  //	キーのビット定数.
  this.pause = (1 << 0);
  this.step = (1 << 1);

  this.mouse_button = (1 << 8);

  //	キーの状態.
  this.up = false;
  this.down = false;
  this.left = false;
  this.right = false;

  this.mouse_x = 0;
  this.mouse_y = 0;

  //	レベルとトリガー.
  this.level = 0;
  this.old_level = 0;
  this.trigger = 0;

  //	キー入力値の初期化.
  this.initialize = function() {
    this.up = false;
    this.down = false;
    this.left = false;
    this.right = false;

    //	キー入力イベントを設定.
    var self = this;

  	//	キーが押された.
  	document.onkeydown = function(ev) {
  		var event = ev? ev : window.event;
  		return self.updateKeyState(event.keyCode, true);
  	};

  	//	キーが離された.
  	document.onkeyup = function(ev) {
  		var event = ev? ev : window.event;
  		return self.updateKeyState(event.keyCode, false);
  	};
  };

  //	キー入力の更新.
  this.updateKeyState = function(keycode, flag) {
    //		console.log(keycode + ":" + flag);

    switch (keycode) {
      case 37:
        this.left = flag;
        return false;
      case 38:
        this.up = flag;
        return false;
      case 39:
        this.right = flag;
        return false;
      case 40:
        this.down = flag;
        return false;

      case 80:
        flag ? this.level |= this.pause : this.level &= ~this.pause;
        return false; //	p
      case 83:
        flag ? this.level |= this.step : this.level &= ~this.step;
        return false; //	s
    }

    return true;
  };

  //	マウスクリック.
  this.updateMouseState = function(ev, flag) {
    //	とりあえず何か押されてたら記録する.
    flag ? this.level |= this.mouse_button : this.level &= ~this.mouse_button;

    //	位置.
    var rect = ev.target.getBoundingClientRect();
    this.mouse_x = ev.clientX - rect.left;
    this.mouse_y = ev.clientY - rect.top;

    //		console.log(this.mouse_x + " , " + this.mouse_y);

    return false;

    //		return true;
  };

  //	毎フレームの実行処理.
  this.update = function() {
    //	トリガーを生成する.
    this.trigger = this.level & (~this.old_level);

    //	キーの状態を保存.
    this.old_level = this.level;
  };
};

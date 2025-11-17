//=====================================================================================
//  UART Simple Verification Environment 
//=====================================================================================

class transaction;
  typedef enum bit {WRITE = 1'b0, READ = 1'b1} oper_type;

  randc oper_type oper;      // cyclic randomization of read/write
  rand bit [7:0]  dintx;     // data to be transmitted / expected on receive

  bit       rx;
  bit       tx;
  bit       newd;
  bit       donetx;
  bit       donerx;
  bit [7:0] doutrx;

  function transaction copy();
    copy = new();
    copy.oper    = this.oper;
    copy.dintx   = this.dintx;
    copy.rx      = this.rx;
    copy.tx      = this.tx;
    copy.newd    = this.newd;
    copy.donetx  = this.donetx;
    copy.donerx  = this.donerx;
    copy.doutrx  = this.doutrx;
  endfunction
endclass

//=====================================================================================
class generator;
  transaction                tr;
  mailbox #(transaction)     mbx;
  event                      done;
  int                        count = 0;
  event                      drvnext;
  event                      sconext;

  function new(mailbox #(transaction) mbx);
    this.mbx = mbx;
    tr = new();
  endfunction

  task run();
    repeat(count) begin
      assert(tr.randomize()) else $fatal(1, "[GEN] Randomization failed");
      mbx.put(tr.copy());
      $display("[%0t] [GEN] Oper = %s, Data = %0d", $time,
               (tr.oper == transaction::WRITE) ? "WRITE" : "READ", tr.dintx);

      @(drvnext);
      @(sconext);
    end
    ->done;
  endtask
endclass

//=====================================================================================
//=====================================================================================
//  DRIVER – Fixed declarations + odd parity
//=====================================================================================
class driver;
  virtual uart_if            vif;
  transaction                tr;
  mailbox #(transaction)     mbx;
  mailbox #(bit[7:0])        mbxds;
  event                      drvnext;

  function new(mailbox #(bit[7:0]) mbxds, mailbox #(transaction) mbx);
    this.mbxds = mbxds;
    this.mbx   = mbx;
  endfunction

  task reset();
    vif.rst   <= 1;
    vif.rx    <= 1;
    vif.newd  <= 0;
    vif.dintx <= '0;
    repeat(10) @(posedge vif.uclktx);
    vif.rst   <= 0;
    $display("[%0t] [DRV] Reset done", $time);
  endtask

  // Odd parity function (safe syntax)
  function logic odd_parity(input logic [7:0] data);
    return ~(^data);        // even parity XOR → invert → odd parity
  endfunction

  task run();
    forever begin
      mbx.get(tr);

      if(tr.oper == transaction::WRITE) begin
        // ---------- TX PATH  ----------
        @(posedge vif.uclktx);
        vif.newd  <= 1;
        vif.rx    <= 1;
        vif.dintx <= tr.dintx;
        @(posedge vif.uclktx);
        vif.newd  <= 0;

        mbxds.put(tr.dintx);
        $display("[%0t] [DRV] TX → Data = %0d", $time, tr.dintx);
        wait(vif.donetx == 1'b1);
        ->drvnext;
      end
      else begin
        // ---------- RX PATH WITH PARITY ----------
        logic [7:0] data_to_send;
        logic       parity_bit;

        data_to_send = tr.dintx;
        parity_bit   = odd_parity(data_to_send);

        @(posedge vif.uclkrx); vif.rx <= 0;             // Start bit
        repeat(8) begin
          @(posedge vif.uclkrx);
          vif.rx <= data_to_send[0];
          data_to_send = data_to_send >> 1;
        end
        @(posedge vif.uclkrx); vif.rx <= parity_bit;    // Parity bit
        @(posedge vif.uclkrx); vif.rx <= 1;             // Stop bit

        mbxds.put(tr.dintx);
        $display("[%0t] [DRV] RX driving data=%0d (parity=%b)", $time, tr.dintx, parity_bit);
        wait(vif.donerx == 1'b1);
        ->drvnext;
      end
    end
  endtask
endclass
//=====================================================================================
//=====================================================================================
//  MONITOR –
//=====================================================================================
class monitor;
  virtual uart_if            vif;
  mailbox #(bit[7:0])        mbx;

  // Declare all variables at class level → no syntax errors
  logic [7:0] captured_data;
  logic       rcvd_parity;
  logic       exp_parity;
  logic       stop_bit;

  function new(mailbox #(bit[7:0]) mbx);
    this.mbx = mbx;
  endfunction

  // Safe odd parity function
  function logic odd_parity(input logic [7:0] d);
    return ~(^d);
  endfunction

  task run();
    fork
      // ==================== TX MONITOR (WITH PARITY & STOP CHECK) ====================
      forever begin
        // Wait for start bit
        @(posedge vif.uclktx);
        if (vif.tx === 1'b0) begin
          $display("[%0t] [MON] Start bit detected", $time);

          captured_data = 0;

          // Sample 8 data bits (LSB first)
          repeat(8) begin
            @(posedge vif.uclktx);
            captured_data = {vif.tx, captured_data[7:1]};
          end

          // Sample parity bit
          @(posedge vif.uclktx);
          rcvd_parity = vif.tx;
          exp_parity  = odd_parity(captured_data);

          if (rcvd_parity !== exp_parity)
            $error("[%0t] [MON] PARITY ERROR! Data=%h  Exp=%b  Got=%b",
                   $time, captured_data, exp_parity, rcvd_parity);
          else
            $display("[%0t] [MON] Parity correct (got %b)", $time, rcvd_parity);

          // Sample stop bit
          @(posedge vif.uclktx);
          stop_bit = vif.tx;
          if (stop_bit !== 1'b1)
            $error("[%0t] [MON] STOP BIT ERROR! Got %b", $time, stop_bit);
          else
            $display("[%0t] [MON] Stop bit correct", $time);

          $display("[%0t] [MON] TX Captured Data = %0d (%h)", $time, captured_data, captured_data);
          mbx.put(captured_data);
        end
      end

      // ==================== RX MONITOR (unchanged) ====================
      forever begin
        @(posedge vif.donerx);
        #1;
        captured_data = vif.doutrx;
        $display("[%0t] [MON] RX Captured Data = %0d", $time, captured_data);
        mbx.put(captured_data);
      end
    join
  endtask
endclass

//=====================================================================================
class scoreboard;
  mailbox #(bit[7:0]) mbxds;   // driver   → scoreboard (expected)
  mailbox #(bit[7:0]) mbxms;   // monitor  → scoreboard (actual)
  event               sconext;

  function new(mailbox #(bit[7:0]) mbxds, mailbox #(bit[7:0]) mbxms);
    this.mbxds = mbxds;
    this.mbxms = mbxms;
  endfunction

  task run();
    bit [7:0] exp, act;
    forever begin
      mbxds.get(exp);
      mbxms.get(act);

      $display("[%0t] [SCO] Expected = %0d  |  Actual = %0d", $time, exp, act);
      if(exp === act)
        $display("   [PASS]");
      else
        $display("   [FAIL]");
      ->sconext;
    end
  endtask
endclass

//=====================================================================================
class environment;
  generator   gen;
  driver      drv;
  monitor     mon;
  scoreboard  sco;

  mailbox #(transaction) gd_mbx;
  mailbox #(bit[7:0])    ds_mbx, ms_mbx;

  event nextgd, nextgs;

  virtual uart_if vif;

  function new(virtual uart_if vif);
    this.vif = vif;

    gd_mbx = new();
    ds_mbx = new();
    ms_mbx = new();

    gen = new(gd_mbx);
    drv = new(ds_mbx, gd_mbx);
    mon = new(ms_mbx);
    sco = new(ds_mbx, ms_mbx);

    drv.vif = vif;
    mon.vif = vif;

    // connect events
    gen.drvnext = nextgd;
    drv.drvnext = nextgd;
    gen.sconext = nextgs;
    sco.sconext = nextgs;
  endfunction

  task pre_test();  drv.reset(); endtask
  task post_test();
    wait(gen.done.triggered);
    #200;
    $display("[%0t] Simulation complete", $time);
  endtask

  task test();
    fork
      gen.run();
      drv.run();
      mon.run();
      sco.run();
    join_any
  endtask

  task run();
    pre_test();
    test();
    post_test();
  endtask
endclass

//=====================================================================================
/*interface uart_if;
  logic clk, rst;
  logic rx, tx;
  logic newd;
  logic [7:0] dintx;
  logic donetx, donerx;
  logic [7:0] doutrx;

  logic uclktx, uclkrx;   // internal baud clocks from DUT
endinterface
*/
//=====================================================================================
module tb;
  uart_if vif();

  uart_top #(1000000,9600) dut (
    .clk    (vif.clk),
    .rst    (vif.rst),
    .rx     (vif.rx),
    .newd   (vif.newd),
    .dintx  (vif.dintx),
    .tx     (vif.tx),
    .donetx (vif.donetx),
    .doutrx (vif.doutrx),
    .donerx (vif.donerx)
  );

  // Clock generation
  initial vif.clk = 0;
  always #10 vif.clk = ~vif.clk;

  // Connect internal UART clocks
  assign vif.uclktx = dut.utx.uclk;
  assign vif.uclkrx = dut.rtx.uclk;

  environment env;

  initial begin
    env = new(vif);
    env.gen.count = 10;      // 10 random transactions
    env.run();
    $finish;
  end

  initial begin
    $dumpfile("uart_tb.vcd");
    $dumpvars(0, tb);
  end
endmodule

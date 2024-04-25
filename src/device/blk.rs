//! VirtIO 块设备驱动程序

use crate::hal::Hal;
use crate::queue::VirtQueue;
use crate::transport::Transport;
use crate::volatile::{volread, Volatile};
use crate::{Error, Result};
use bitflags::bitflags;
use log::info;
use zerocopy::{AsBytes, FromBytes, FromZeroes};

const QUEUE: u16 = 0;
const QUEUE_SIZE: u16 = 16;
const SUPPORTED_FEATURES: BlkFeature = BlkFeature::RO
    .union(BlkFeature::FLUSH)
    .union(BlkFeature::RING_INDIRECT_DESC)
    .union(BlkFeature::RING_EVENT_IDX);

/// VirtIO 块设备的驱动程序
///
/// 简单的虚拟块设备，例如磁盘
///
/// 读取和写入请求（以及其他特殊请求）被放置在队列中，并由设备服务（可能是无序的），除非另有说明。
///
/// # Example
///
/// ```
/// # use virtio_drivers::{Error, Hal};
/// # use virtio_drivers::transport::Transport;
/// use virtio_drivers::device::blk::{VirtIOBlk, SECTOR_SIZE};
///
/// # fn example<HalImpl: Hal, T: Transport>(transport: T) -> Result<(), Error> {
/// let mut disk = VirtIOBlk::<HalImpl, _>::new(transport)?;
///
/// println!("VirtIO block device: {} kB", disk.capacity() * SECTOR_SIZE as u64 / 2);
///
/// // 读取扇区 0，然后将其复制到扇区 1
/// let mut buf = [0; SECTOR_SIZE];
/// disk.read_blocks(0, &mut buf)?;
/// disk.write_blocks(1, &buf)?;
/// # Ok(())
/// # }
/// ```
pub struct VirtIOBlk<H: Hal, T: Transport> {
    transport: T,
    queue: VirtQueue<H, { QUEUE_SIZE as usize }>,
    capacity: u64,
    negotiated_features: BlkFeature,
}

impl<H: Hal, T: Transport> VirtIOBlk<H, T> {
    /// 创建一个新的VirtIO-Blk驱动程序
    pub fn new(mut transport: T) -> Result<Self> {
        let negotiated_features = transport.begin_init(SUPPORTED_FEATURES);

        // 读取配置
        let config = transport.config_space::<BlkConfig>()?;
        info!("config: {:?}", config);
        // Safe 因为 config 是指向设备配置空间的有效指针
        let capacity = unsafe {
            volread!(config, capacity_low) as u64 | (volread!(config, capacity_high) as u64) << 32
        };
        info!("found a block device of size {}KB", capacity / 2);

        let queue = VirtQueue::new(
            &mut transport,
            QUEUE,
            negotiated_features.contains(BlkFeature::RING_INDIRECT_DESC),
            negotiated_features.contains(BlkFeature::RING_EVENT_IDX),
        )?;
        transport.finish_init();

        Ok(VirtIOBlk {
            transport,
            queue,
            capacity,
            negotiated_features,
        })
    }

    /// 获取块设备的容量, in 512 byte ([`SECTOR_SIZE`]) sectors.
    pub fn capacity(&self) -> u64 {
        self.capacity
    }

    /// 如果块设备是只读的，则返回 true；如果允许写入，则返回 false。.
    pub fn readonly(&self) -> bool {
        self.negotiated_features.contains(BlkFeature::RO)
    }

    /// 如果有挂起的中断则确认
    ///
    /// 如果需要确认则返回true
    pub fn ack_interrupt(&mut self) -> bool {
        self.transport.ack_interrupt()
    }

    /// 将给定的请求发送到设备，并等待响应，不带额外数据。
    fn request(&mut self, request: BlkReq) -> Result {
        let mut resp = BlkResp::default();
        self.queue.add_notify_wait_pop(
            &[request.as_bytes()],
            &mut [resp.as_bytes_mut()],
            &mut self.transport,
        )?;
        resp.status.into()
    }

    /// 将给定的请求发送到设备，并等待响应，包括给定的数据。
    fn request_read(&mut self, request: BlkReq, data: &mut [u8]) -> Result {
        let mut resp = BlkResp::default();
        self.queue.add_notify_wait_pop(
            &[request.as_bytes()],
            &mut [data, resp.as_bytes_mut()],
            &mut self.transport,
        )?;
        resp.status.into()
    }

    /// 将给定的请求和数据发送到设备，并等待响应。
    fn request_write(&mut self, request: BlkReq, data: &[u8]) -> Result {
        let mut resp = BlkResp::default();
        self.queue.add_notify_wait_pop(
            &[request.as_bytes(), data],
            &mut [resp.as_bytes_mut()],
            &mut self.transport,
        )?;
        resp.status.into()
    }

    /// 请求设备将任何待处理的写操作刷新到存储介质
    ///
    /// 如果设备不支持 VIRTIO_BLK_F_FLUSH 特性，则此操作将被忽略
    pub fn flush(&mut self) -> Result {
        if self.negotiated_features.contains(BlkFeature::FLUSH) {
            self.request(BlkReq {
                type_: ReqType::Flush,
                ..Default::default()
            })
        } else {
            Ok(())
        }
    }

    /// 获取设备ID.
    ///
    /// 将 ID 以 ASCII 格式写入给定的缓冲区中，该缓冲区必须为 20 字节长，并返回已使用的长度
    pub fn device_id(&mut self, id: &mut [u8; 20]) -> Result<usize> {
        self.request_read(
            BlkReq {
                type_: ReqType::GetId,
                ..Default::default()
            },
            id,
        )?;

        let length = id.iter().position(|&x| x == 0).unwrap_or(20);
        Ok(length)
    }

    /// 将一个或多个块读入到给定的缓冲区中
    ///
    /// 缓冲区长度必须是 [`SECTOR_SIZE`] 的非零倍数。
    ///
    /// 阻塞直到读取完成或出现错误。
    pub fn read_blocks(&mut self, block_id: usize, buf: &mut [u8]) -> Result {
        assert_ne!(buf.len(), 0);
        assert_eq!(buf.len() % SECTOR_SIZE, 0);
        self.request_read(
            BlkReq {
                type_: ReqType::In,
                reserved: 0,
                sector: block_id as u64,
            },
            buf,
        )
    }

    /// 提交一个请求来读取一个或多个块，但立即返回而不等待读取完成
    ///
    /// # Arguments
    ///
    /// * `block_id` - 要读取的第一个块的标识符。
    /// * `req` - 驱动程序可以用于发送给设备的请求的缓冲区。
    /// 其内容不重要，因为 `read_blocks_nb` 将对其进行初始化，
    /// 但与其他缓冲区一样，直到相应的 `complete_read_blocks` 调用之前，它需要是有效的（并且未被其他地方使用）。
    /// 其长度必须是 [`SECTOR_SIZE`] 的非零倍数。
    /// * `buf` - 要读取的块应该被读入的内存缓冲区。
    /// * `resp` - 调用者提供的一个可变引用，用于保存请求的状态。调用者只能在请求完成后安全地读取该变量。
    ///
    /// # Usage
    ///
    /// 它将向VirtIO块设备提交请求，并返回一个标识第一个描述符在链中位置的令牌。
    /// 如果没有足够的描述符可供分配，则返回 [`Error::QueueFull`]。
    ///
    /// 然后调用者可以使用返回的令牌调用 peek_used 来检查设备是否已经完成处理请求。
    /// 一旦设备完成处理，调用者必须在读取响应之前使用相同的缓冲区调用 complete_read_blocks。
    ///
    /// ```
    /// # use virtio_drivers::{Error, Hal};
    /// # use virtio_drivers::device::blk::VirtIOBlk;
    /// # use virtio_drivers::transport::Transport;
    /// use virtio_drivers::device::blk::{BlkReq, BlkResp, RespStatus};
    ///
    /// # fn example<H: Hal, T: Transport>(blk: &mut VirtIOBlk<H, T>) -> Result<(), Error> {
    /// let mut request = BlkReq::default();
    /// let mut buffer = [0; 512];
    /// let mut response = BlkResp::default();
    /// let token = unsafe { blk.read_blocks_nb(42, &mut request, &mut buffer, &mut response) }?;
    ///
    /// // 等待中断以通知我们请求已完成。
    /// assert_eq!(blk.peek_used(), Some(token));
    ///
    /// unsafe {
    ///   blk.complete_read_blocks(token, &request, &mut buffer, &mut response)?;
    /// }
    /// if response.status() == RespStatus::OK {
    ///   println!("Successfully read block.");
    /// } else {
    ///   println!("Error {:?} reading block.", response.status());
    /// }
    /// # Ok(())
    /// # }
    /// ```
    ///
    /// # Safety
    ///
    /// 即使在此方法返回后，`req`、`buf` 和 `resp` 仍由底层的 VirtIO 块设备借用。
    /// 因此，调用者有责任确保在请求完成之前不访问它们，以避免数据竞争。
    pub unsafe fn read_blocks_nb(
        &mut self,
        block_id: usize,
        req: &mut BlkReq,
        buf: &mut [u8],
        resp: &mut BlkResp,
    ) -> Result<u16> {
        assert_ne!(buf.len(), 0);
        assert_eq!(buf.len() % SECTOR_SIZE, 0);
        *req = BlkReq {
            type_: ReqType::In,
            reserved: 0,
            sector: block_id as u64,
        };
        let token = self
            .queue
            .add(&[req.as_bytes()], &mut [buf, resp.as_bytes_mut()])?;
        if self.queue.should_notify() {
            self.transport.notify(QUEUE);
        }
        Ok(token)
    }

    /// 完成由 `read_blocks_nb` 启动的读取操作。
    ///
    /// # Safety
    ///
    /// 当 `read_blocks_nb` 返回令牌时，必须再次传递相同的缓冲区，作为参数传递给此方法。
    pub unsafe fn complete_read_blocks(
        &mut self,
        token: u16,
        req: &BlkReq,
        buf: &mut [u8],
        resp: &mut BlkResp,
    ) -> Result<()> {
        self.queue
            .pop_used(token, &[req.as_bytes()], &mut [buf, resp.as_bytes_mut()])?;
        resp.status.into()
    }

    /// 将给定缓冲区的内容写入到一个或多个块中。
    ///
    /// 缓冲区的长度必须是 [`SECTOR_SIZE`] 的非零倍数。
    ///
    /// 阻塞直到写入完成或出现错误。
    pub fn write_blocks(&mut self, block_id: usize, buf: &[u8]) -> Result {
        assert_ne!(buf.len(), 0);
        assert_eq!(buf.len() % SECTOR_SIZE, 0);
        self.request_write(
            BlkReq {
                type_: ReqType::Out,
                sector: block_id as u64,
                ..Default::default()
            },
            buf,
        )
    }

    /// 提交一个请求来写入一个或多个块，但立即返回而不等待写入完成。
    ///
    /// # Arguments
    ///
    /// * `block_id` - 要写入的第一个块的标识符。
    /// * `req` - 驱动程序可以用于发送给设备的请求的缓冲区。其内容不重要，因为 `read_blocks_nb` 将对其进行初始化，
    /// 但与其他缓冲区一样，直到相应的 `complete_write_blocks` 调用之前，它需要是有效的（并且未被其他地方使用）。
    /// * `buf` - 内存中包含要写入块的数据的缓冲区。其长度必须是 [`SECTOR_SIZE`] 的非零倍数。
    /// * `resp` - 调用者提供的一个可变引用，用于保存请求的状态。调用者只能在请求完成后安全地读取该变量。
    ///
    /// # Usage
    ///
    /// 见 [VirtIOBlk::read_blocks_nb].
    ///
    /// # Safety
    ///
    /// 见  [VirtIOBlk::read_blocks_nb].
    pub unsafe fn write_blocks_nb(
        &mut self,
        block_id: usize,
        req: &mut BlkReq,
        buf: &[u8],
        resp: &mut BlkResp,
    ) -> Result<u16> {
        assert_ne!(buf.len(), 0);
        assert_eq!(buf.len() % SECTOR_SIZE, 0);
        *req = BlkReq {
            type_: ReqType::Out,
            reserved: 0,
            sector: block_id as u64,
        };
        let token = self
            .queue
            .add(&[req.as_bytes(), buf], &mut [resp.as_bytes_mut()])?;
        if self.queue.should_notify() {
            self.transport.notify(QUEUE);
        }
        Ok(token)
    }

    /// 完成由 `write_blocks_nb` 启动的写入操作。
    ///
    /// # Safety
    ///
    /// 当 `write_blocks_nb` 返回令牌时，必须再次传递与传递给该方法的相同的缓冲区。
    pub unsafe fn complete_write_blocks(
        &mut self,
        token: u16,
        req: &BlkReq,
        buf: &[u8],
        resp: &mut BlkResp,
    ) -> Result<()> {
        self.queue
            .pop_used(token, &[req.as_bytes(), buf], &mut [resp.as_bytes_mut()])?;
        resp.status.into()
    }

    /// 从已使用的环中获取下一个已完成请求的令牌，并返回它，而不从已使用的环中删除它。
    /// 如果没有待处理的已完成请求，则返回 `None`。
    pub fn peek_used(&mut self) -> Option<u16> {
        self.queue.peek_used()
    }

    /// 返回设备的 VirtQueue 的大小。
    ///
    /// 这可以用来告诉调用者需要监视多少个通道。
    pub fn virt_queue_size(&self) -> u16 {
        QUEUE_SIZE
    }
}

impl<H: Hal, T: Transport> Drop for VirtIOBlk<H, T> {
    fn drop(&mut self) {
        // 清除任何指向 DMA 区域的指针，以防止设备在它们被释放后尝试访问它们。
        self.transport.queue_unset(QUEUE);
    }
}

#[repr(C)]
struct BlkConfig {
    /// 512字节扇区的数量
    capacity_low: Volatile<u32>,
    capacity_high: Volatile<u32>,
    size_max: Volatile<u32>,
    seg_max: Volatile<u32>,
    cylinders: Volatile<u16>,
    heads: Volatile<u8>,
    sectors: Volatile<u8>,
    blk_size: Volatile<u32>,
    physical_block_exp: Volatile<u8>,
    alignment_offset: Volatile<u8>,
    min_io_size: Volatile<u16>,
    opt_io_size: Volatile<u32>,
    // ... ignored
}

/// VirtIO块设备请求。
#[repr(C)]
#[derive(AsBytes, Debug)]
pub struct BlkReq {
    type_: ReqType,
    reserved: u32,
    sector: u64,
}

impl Default for BlkReq {
    fn default() -> Self {
        Self {
            type_: ReqType::In,
            reserved: 0,
            sector: 0,
        }
    }
}

/// VirtIOBlk 请求的响应。
#[repr(C)]
#[derive(AsBytes, Debug, FromBytes, FromZeroes)]
pub struct BlkResp {
    status: RespStatus,
}

impl BlkResp {
    /// 返回 VirtIOBlk 请求的状态。
    pub fn status(&self) -> RespStatus {
        self.status
    }
}

#[repr(u32)]
#[derive(AsBytes, Debug)]
enum ReqType {
    In = 0,
    Out = 1,
    Flush = 4,
    GetId = 8,
    GetLifetime = 10,
    Discard = 11,
    WriteZeroes = 13,
    SecureErase = 14,
}

/// VirtIOBlk 请求的状态。
#[repr(transparent)]
#[derive(AsBytes, Copy, Clone, Debug, Eq, FromBytes, FromZeroes, PartialEq)]
pub struct RespStatus(u8);

impl RespStatus {
    /// Ok.
    pub const OK: RespStatus = RespStatus(0);
    /// IoErr.
    pub const IO_ERR: RespStatus = RespStatus(1);
    /// Unsupported yet.
    pub const UNSUPPORTED: RespStatus = RespStatus(2);
    /// Not ready.
    pub const NOT_READY: RespStatus = RespStatus(3);
}

impl From<RespStatus> for Result {
    fn from(status: RespStatus) -> Self {
        match status {
            RespStatus::OK => Ok(()),
            RespStatus::IO_ERR => Err(Error::IoError),
            RespStatus::UNSUPPORTED => Err(Error::Unsupported),
            RespStatus::NOT_READY => Err(Error::NotReady),
            _ => Err(Error::IoError),
        }
    }
}

impl Default for BlkResp {
    fn default() -> Self {
        BlkResp {
            status: RespStatus::NOT_READY,
        }
    }
}

/// VirtIO 块设备的标准扇区大小。数据以这个大小的倍数进行读取和写入。
pub const SECTOR_SIZE: usize = 512;

bitflags! {
    #[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
    struct BlkFeature: u64 {
        /// 设备支持请求屏障 (legacy)
        const BARRIER       = 1 << 0;
        /// 任何单个段的最大大小在 `size_max` 中。
        const SIZE_MAX      = 1 << 1;
        /// 一个请求中的最大段数在 `seg_max` 中。
        const SEG_MAX       = 1 << 2;
        /// 在 geometry 中指定了磁盘样式的几何结构。
        const GEOMETRY      = 1 << 4;
        /// 设备是只读的。
        const RO            = 1 << 5;
        /// 磁盘的块大小在 `blk_size` 中。
        const BLK_SIZE      = 1 << 6;
        /// 设备支持 SCSI 包命令（legacy）。
        const SCSI          = 1 << 7;
        /// 缓存刷新命令支持。
        const FLUSH         = 1 << 9;
        /// 设备提供有关最佳 I/O 对齐的信息。
        const TOPOLOGY      = 1 << 10;
        /// 设备可以在写回和写穿透模式之间切换其缓存。
        const CONFIG_WCE    = 1 << 11;
        /// 设备支持多队列。
        const MQ            = 1 << 12;
        /// 设备可以支持丢弃命令，在 `max_discard_sectors` 中指定最大丢弃扇区大小，
        /// 在 `max_discard_seg` 中指定最大丢弃段数。
        const DISCARD       = 1 << 13;
        /// 设备可以支持写零命令，在 `max_write_zeroes_sectors` 中指定最大写零扇区大小，
        /// 在 `max_write_zeroes_seg` 中指定最大写零段数。
        const WRITE_ZEROES  = 1 << 14;
        /// 设备支持提供存储寿命信息。
        const LIFETIME      = 1 << 15;
        /// 设备可以支持安全擦除命令。
        const SECURE_ERASE  = 1 << 16;

        // 设备独立的特性
        const NOTIFY_ON_EMPTY       = 1 << 24; // legacy
        const ANY_LAYOUT            = 1 << 27; // legacy
        const RING_INDIRECT_DESC    = 1 << 28;
        const RING_EVENT_IDX        = 1 << 29;
        const UNUSED                = 1 << 30; // legacy
        const VERSION_1             = 1 << 32; // detect legacy

        // 自 VirtIO v1.1 起支持以下功能。
        const ACCESS_PLATFORM       = 1 << 33;
        const RING_PACKED           = 1 << 34;
        const IN_ORDER              = 1 << 35;
        const ORDER_PLATFORM        = 1 << 36;
        const SR_IOV                = 1 << 37;
        const NOTIFICATION_DATA     = 1 << 38;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        hal::fake::FakeHal,
        transport::{
            fake::{FakeTransport, QueueStatus, State},
            DeviceType,
        },
    };
    use alloc::{sync::Arc, vec};
    use core::{mem::size_of, ptr::NonNull};
    use std::{sync::Mutex, thread};

    #[test]
    fn config() {
        let mut config_space = BlkConfig {
            capacity_low: Volatile::new(0x42),
            capacity_high: Volatile::new(0x02),
            size_max: Volatile::new(0),
            seg_max: Volatile::new(0),
            cylinders: Volatile::new(0),
            heads: Volatile::new(0),
            sectors: Volatile::new(0),
            blk_size: Volatile::new(0),
            physical_block_exp: Volatile::new(0),
            alignment_offset: Volatile::new(0),
            min_io_size: Volatile::new(0),
            opt_io_size: Volatile::new(0),
        };
        let state = Arc::new(Mutex::new(State {
            queues: vec![QueueStatus::default()],
            ..Default::default()
        }));
        let transport = FakeTransport {
            device_type: DeviceType::Block,
            max_queue_size: QUEUE_SIZE.into(),
            device_features: BlkFeature::RO.bits(),
            config_space: NonNull::from(&mut config_space),
            state: state.clone(),
        };
        let blk = VirtIOBlk::<FakeHal, FakeTransport<BlkConfig>>::new(transport).unwrap();

        assert_eq!(blk.capacity(), 0x02_0000_0042);
        assert_eq!(blk.readonly(), true);
    }

    #[test]
    fn read() {
        let mut config_space = BlkConfig {
            capacity_low: Volatile::new(66),
            capacity_high: Volatile::new(0),
            size_max: Volatile::new(0),
            seg_max: Volatile::new(0),
            cylinders: Volatile::new(0),
            heads: Volatile::new(0),
            sectors: Volatile::new(0),
            blk_size: Volatile::new(0),
            physical_block_exp: Volatile::new(0),
            alignment_offset: Volatile::new(0),
            min_io_size: Volatile::new(0),
            opt_io_size: Volatile::new(0),
        };
        let state = Arc::new(Mutex::new(State {
            queues: vec![QueueStatus::default()],
            ..Default::default()
        }));
        let transport = FakeTransport {
            device_type: DeviceType::Block,
            max_queue_size: QUEUE_SIZE.into(),
            device_features: BlkFeature::RING_INDIRECT_DESC.bits(),
            config_space: NonNull::from(&mut config_space),
            state: state.clone(),
        };
        let mut blk = VirtIOBlk::<FakeHal, FakeTransport<BlkConfig>>::new(transport).unwrap();

        // Start a thread to simulate the device waiting for a read request.
        let handle = thread::spawn(move || {
            println!("Device waiting for a request.");
            State::wait_until_queue_notified(&state, QUEUE);
            println!("Transmit queue was notified.");

            state
                .lock()
                .unwrap()
                .read_write_queue::<{ QUEUE_SIZE as usize }>(QUEUE, |request| {
                    assert_eq!(
                        request,
                        BlkReq {
                            type_: ReqType::In,
                            reserved: 0,
                            sector: 42
                        }
                        .as_bytes()
                    );

                    let mut response = vec![0; SECTOR_SIZE];
                    response[0..9].copy_from_slice(b"Test data");
                    response.extend_from_slice(
                        BlkResp {
                            status: RespStatus::OK,
                        }
                        .as_bytes(),
                    );

                    response
                });
        });

        // Read a block from the device.
        let mut buffer = [0; 512];
        blk.read_blocks(42, &mut buffer).unwrap();
        assert_eq!(&buffer[0..9], b"Test data");

        handle.join().unwrap();
    }

    #[test]
    fn write() {
        let mut config_space = BlkConfig {
            capacity_low: Volatile::new(66),
            capacity_high: Volatile::new(0),
            size_max: Volatile::new(0),
            seg_max: Volatile::new(0),
            cylinders: Volatile::new(0),
            heads: Volatile::new(0),
            sectors: Volatile::new(0),
            blk_size: Volatile::new(0),
            physical_block_exp: Volatile::new(0),
            alignment_offset: Volatile::new(0),
            min_io_size: Volatile::new(0),
            opt_io_size: Volatile::new(0),
        };
        let state = Arc::new(Mutex::new(State {
            queues: vec![QueueStatus::default()],
            ..Default::default()
        }));
        let transport = FakeTransport {
            device_type: DeviceType::Block,
            max_queue_size: QUEUE_SIZE.into(),
            device_features: BlkFeature::RING_INDIRECT_DESC.bits(),
            config_space: NonNull::from(&mut config_space),
            state: state.clone(),
        };
        let mut blk = VirtIOBlk::<FakeHal, FakeTransport<BlkConfig>>::new(transport).unwrap();

        // Start a thread to simulate the device waiting for a write request.
        let handle = thread::spawn(move || {
            println!("Device waiting for a request.");
            State::wait_until_queue_notified(&state, QUEUE);
            println!("Transmit queue was notified.");

            state
                .lock()
                .unwrap()
                .read_write_queue::<{ QUEUE_SIZE as usize }>(QUEUE, |request| {
                    assert_eq!(
                        &request[0..size_of::<BlkReq>()],
                        BlkReq {
                            type_: ReqType::Out,
                            reserved: 0,
                            sector: 42
                        }
                        .as_bytes()
                    );
                    let data = &request[size_of::<BlkReq>()..];
                    assert_eq!(data.len(), SECTOR_SIZE);
                    assert_eq!(&data[0..9], b"Test data");

                    let mut response = Vec::new();
                    response.extend_from_slice(
                        BlkResp {
                            status: RespStatus::OK,
                        }
                        .as_bytes(),
                    );

                    response
                });
        });

        // Write a block to the device.
        let mut buffer = [0; 512];
        buffer[0..9].copy_from_slice(b"Test data");
        blk.write_blocks(42, &mut buffer).unwrap();

        // Request to flush should be ignored as the device doesn't support it.
        blk.flush().unwrap();

        handle.join().unwrap();
    }

    #[test]
    fn flush() {
        let mut config_space = BlkConfig {
            capacity_low: Volatile::new(66),
            capacity_high: Volatile::new(0),
            size_max: Volatile::new(0),
            seg_max: Volatile::new(0),
            cylinders: Volatile::new(0),
            heads: Volatile::new(0),
            sectors: Volatile::new(0),
            blk_size: Volatile::new(0),
            physical_block_exp: Volatile::new(0),
            alignment_offset: Volatile::new(0),
            min_io_size: Volatile::new(0),
            opt_io_size: Volatile::new(0),
        };
        let state = Arc::new(Mutex::new(State {
            queues: vec![QueueStatus::default()],
            ..Default::default()
        }));
        let transport = FakeTransport {
            device_type: DeviceType::Block,
            max_queue_size: QUEUE_SIZE.into(),
            device_features: (BlkFeature::RING_INDIRECT_DESC | BlkFeature::FLUSH).bits(),
            config_space: NonNull::from(&mut config_space),
            state: state.clone(),
        };
        let mut blk = VirtIOBlk::<FakeHal, FakeTransport<BlkConfig>>::new(transport).unwrap();

        // Start a thread to simulate the device waiting for a flush request.
        let handle = thread::spawn(move || {
            println!("Device waiting for a request.");
            State::wait_until_queue_notified(&state, QUEUE);
            println!("Transmit queue was notified.");

            state
                .lock()
                .unwrap()
                .read_write_queue::<{ QUEUE_SIZE as usize }>(QUEUE, |request| {
                    assert_eq!(
                        request,
                        BlkReq {
                            type_: ReqType::Flush,
                            reserved: 0,
                            sector: 0,
                        }
                        .as_bytes()
                    );

                    let mut response = Vec::new();
                    response.extend_from_slice(
                        BlkResp {
                            status: RespStatus::OK,
                        }
                        .as_bytes(),
                    );

                    response
                });
        });

        // Request to flush.
        blk.flush().unwrap();

        handle.join().unwrap();
    }

    #[test]
    fn device_id() {
        let mut config_space = BlkConfig {
            capacity_low: Volatile::new(66),
            capacity_high: Volatile::new(0),
            size_max: Volatile::new(0),
            seg_max: Volatile::new(0),
            cylinders: Volatile::new(0),
            heads: Volatile::new(0),
            sectors: Volatile::new(0),
            blk_size: Volatile::new(0),
            physical_block_exp: Volatile::new(0),
            alignment_offset: Volatile::new(0),
            min_io_size: Volatile::new(0),
            opt_io_size: Volatile::new(0),
        };
        let state = Arc::new(Mutex::new(State {
            queues: vec![QueueStatus::default()],
            ..Default::default()
        }));
        let transport = FakeTransport {
            device_type: DeviceType::Block,
            max_queue_size: QUEUE_SIZE.into(),
            device_features: BlkFeature::RING_INDIRECT_DESC.bits(),
            config_space: NonNull::from(&mut config_space),
            state: state.clone(),
        };
        let mut blk = VirtIOBlk::<FakeHal, FakeTransport<BlkConfig>>::new(transport).unwrap();

        // Start a thread to simulate the device waiting for a flush request.
        let handle = thread::spawn(move || {
            println!("Device waiting for a request.");
            State::wait_until_queue_notified(&state, QUEUE);
            println!("Transmit queue was notified.");

            state
                .lock()
                .unwrap()
                .read_write_queue::<{ QUEUE_SIZE as usize }>(QUEUE, |request| {
                    assert_eq!(
                        request,
                        BlkReq {
                            type_: ReqType::GetId,
                            reserved: 0,
                            sector: 0,
                        }
                        .as_bytes()
                    );

                    let mut response = Vec::new();
                    response.extend_from_slice(b"device_id\0\0\0\0\0\0\0\0\0\0\0");
                    response.extend_from_slice(
                        BlkResp {
                            status: RespStatus::OK,
                        }
                        .as_bytes(),
                    );

                    response
                });
        });

        let mut id = [0; 20];
        let length = blk.device_id(&mut id).unwrap();
        assert_eq!(&id[0..length], b"device_id");

        handle.join().unwrap();
    }
}
